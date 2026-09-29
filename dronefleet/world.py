"""The environment: a 2-D city grid with buildings, pads and no-fly zones.

The airspace is discretised into cells. Buildings are permanent obstacles.
*Pads* (delivery hubs and battery-swap stations) are cells where drones can
land; a landed drone is on the ground and therefore outside the airspace.
Temporary no-fly zones (NFZ) model events such as an emergency-services
operation: they are announced ``nfz_lead_time`` ticks before they activate,
so agents can plan around them (a NOTAM, in aviation terms).
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from typing import Iterable

Cell = tuple[int, int]
INF = float("inf")

MOVES: tuple[Cell, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))


def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


@dataclass(frozen=True)
class NoFlyZone:
    zid: int
    rect: tuple[int, int, int, int]  # x0, y0, x1, y1 inclusive
    announce_t: int
    start_t: int
    end_t: int  # exclusive

    @property
    def cells(self) -> frozenset[Cell]:
        x0, y0, x1, y1 = self.rect
        return frozenset((x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1))

    def contains(self, c: Cell) -> bool:
        x0, y0, x1, y1 = self.rect
        return x0 <= c[0] <= x1 and y0 <= c[1] <= y1

    def active(self, t: int) -> bool:
        return self.start_t <= t < self.end_t


class GridWorld:
    def __init__(
        self,
        width: int,
        height: int,
        blocked: Iterable[Cell],
        hubs: list[Cell],
        stations: list[Cell],
        customers: list[Cell],
        nfzs: Iterable[NoFlyZone] = (),
    ):
        self.width = width
        self.height = height
        self.blocked = frozenset(blocked)
        self.hubs = list(hubs)
        self.stations = list(stations)
        self.customers = list(customers)
        self.pads = frozenset(self.hubs) | frozenset(self.stations)
        self.nfzs: list[NoFlyZone] = sorted(nfzs, key=lambda z: z.announce_t)
        self._nbr_cache: dict[Cell, tuple[Cell, ...]] = {}
        self._dist_cache: dict[Cell, dict[Cell, int]] = {}
        for p in self.pads:
            if p in self.blocked:
                raise ValueError(f"pad {p} is inside a building")

    # ------------------------------------------------------------------ static
    def in_bounds(self, c: Cell) -> bool:
        return 0 <= c[0] < self.width and 0 <= c[1] < self.height

    def is_open(self, c: Cell) -> bool:
        return self.in_bounds(c) and c not in self.blocked

    def neighbors(self, c: Cell) -> tuple[Cell, ...]:
        """4-connected neighbours that are not buildings (cached)."""
        nb = self._nbr_cache.get(c)
        if nb is None:
            nb = tuple(
                (c[0] + dx, c[1] + dy)
                for dx, dy in MOVES
                if self.is_open((c[0] + dx, c[1] + dy))
            )
            self._nbr_cache[c] = nb
        return nb

    def distance_map(self, goal: Cell) -> dict[Cell, int]:
        """Exact obstacle-aware distance from every cell to ``goal`` (BFS).

        Used as the (admissible and consistent) heuristic by the space-time
        planner and for fast energy estimates when bidding. Temporary NFZs are
        ignored here, which keeps the heuristic admissible: extra obstacles can
        only make true paths longer.
        """
        dm = self._dist_cache.get(goal)
        if dm is None:
            dm = {goal: 0}
            q = deque([goal])
            while q:
                c = q.popleft()
                d = dm[c] + 1
                for n in self.neighbors(c):
                    if n not in dm:
                        dm[n] = d
                        q.append(n)
            self._dist_cache[goal] = dm
        return dm

    def dist(self, a: Cell, b: Cell) -> float:
        return self.distance_map(b).get(a, INF)

    # ----------------------------------------------------------------- dynamic
    def known_zones(self, now: int) -> list[NoFlyZone]:
        """Zones an agent can know about at ``now`` that are not yet over."""
        return [z for z in self.nfzs if z.announce_t <= now and z.end_t > now]

    def zones_announced_at(self, t: int) -> list[NoFlyZone]:
        return [z for z in self.nfzs if z.announce_t == t]

    def in_active_nfz(self, c: Cell, t: int) -> bool:
        return any(z.active(t) and z.contains(c) for z in self.nfzs)

    # ----------------------------------------------------------------- display
    def ascii(self, marks: dict[Cell, str] | None = None) -> str:
        marks = marks or {}
        rows = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                c = (x, y)
                if c in marks:
                    row.append(marks[c])
                elif c in self.blocked:
                    row.append("#")
                elif c in self.hubs:
                    row.append("H")
                elif c in self.stations:
                    row.append("S")
                elif c in self.customers:
                    row.append("c")
                else:
                    row.append(".")
            rows.append("".join(row))
        return "\n".join(rows)


# ---------------------------------------------------------------------------
# Procedural city generation
# ---------------------------------------------------------------------------

def _largest_component(width: int, height: int, blocked: set[Cell]) -> set[Cell]:
    seen: set[Cell] = set()
    best: set[Cell] = set()
    for x in range(width):
        for y in range(height):
            s = (x, y)
            if s in blocked or s in seen:
                continue
            comp = {s}
            q = deque([s])
            seen.add(s)
            while q:
                cx, cy = q.popleft()
                for dx, dy in MOVES:
                    n = (cx + dx, cy + dy)
                    if 0 <= n[0] < width and 0 <= n[1] < height and n not in blocked and n not in seen:
                        seen.add(n)
                        comp.add(n)
                        q.append(n)
            if len(comp) > len(best):
                best = comp
    return best


def _spread_sample(rng: random.Random, candidates: list[Cell], k: int, taken: list[Cell]) -> list[Cell]:
    """Farthest-point sampling so pads are spread across the city."""
    chosen: list[Cell] = []
    pool = list(candidates)
    for _ in range(k):
        anchors = taken + chosen
        if not anchors:
            pick = rng.choice(pool)
        else:
            # choose among the top few farthest to keep some randomness
            scored = sorted(pool, key=lambda c: -min(manhattan(c, a) for a in anchors))
            pick = rng.choice(scored[: max(1, len(scored) // 25)])
        chosen.append(pick)
        pool = [c for c in pool if manhattan(c, pick) > 3]
    return chosen


def generate_world(cfg) -> GridWorld:
    rng = random.Random(cfg.seed * 7919 + 17)
    W, H = cfg.width, cfg.height
    blocked: set[Cell] = set()
    target = int(W * H * cfg.building_density)
    guard = 0
    while len(blocked) < target and guard < 10_000:
        guard += 1
        w, h = rng.randint(1, 3), rng.randint(1, 3)
        x0, y0 = rng.randrange(0, W - w + 1), rng.randrange(0, H - h + 1)
        for x in range(x0, x0 + w):
            for y in range(y0, y0 + h):
                blocked.add((x, y))

    # Pads are placed on a coarse interior band so they are reachable from
    # several directions; their 3x3 neighbourhood is cleared of buildings.
    interior = [(x, y) for x in range(2, W - 2) for y in range(2, H - 2)]
    hubs = _spread_sample(rng, interior, cfg.n_hubs, [])
    stations = _spread_sample(rng, interior, cfg.n_stations, hubs)
    for p in hubs + stations:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                blocked.discard((p[0] + dx, p[1] + dy))

    # Remove unreachable pockets: fill everything outside the main component.
    comp = _largest_component(W, H, blocked)
    blocked = {(x, y) for x in range(W) for y in range(H)} - comp
    pads = set(hubs) | set(stations)
    assert pads <= comp, "pad generation produced an unreachable pad"

    cust_pool = [c for c in comp if c not in pads and min(manhattan(c, p) for p in pads) >= 3]
    rng.shuffle(cust_pool)
    customers = sorted(cust_pool[: cfg.n_customers])

    nfzs = [NoFlyZone(i, tuple(r), a, s, e) for i, (a, s, e, r) in enumerate(cfg.nfz_events)]
    if cfg.auto_nfz and not nfzs:
        nfzs = _generate_nfzs(rng, cfg, W, H, pads)
    return GridWorld(W, H, blocked, hubs, stations, customers, nfzs)


def _generate_nfzs(rng: random.Random, cfg, W: int, H: int, pads: set[Cell]) -> list[NoFlyZone]:
    zones = []
    horizon = max(cfg.order_until, 200)
    starts = sorted(rng.sample(range(80, horizon - 40), 2)) if horizon > 140 else []
    for i, start in enumerate(starts):
        for _ in range(200):
            w, h = rng.randint(4, 6), rng.randint(3, 5)
            x0, y0 = rng.randrange(1, W - w - 1), rng.randrange(1, H - h - 1)
            rect = (x0, y0, x0 + w - 1, y0 + h - 1)
            # never close a pad (or its approach cells)
            if any(x0 - 1 <= p[0] <= rect[2] + 1 and y0 - 1 <= p[1] <= rect[3] + 1 for p in pads):
                continue
            dur = rng.randint(50, 80)
            zones.append(NoFlyZone(i, rect, start - cfg.nfz_lead_time, start, start + dur))
            break
    return zones
