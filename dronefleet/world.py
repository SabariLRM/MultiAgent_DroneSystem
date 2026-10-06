"""The environment: a layered city airspace with buildings, pads and no-fly zones.

The city is a 2-D grid of *sites* ``(x, y)``. Above it, the airspace is
discretised into *cells* ``(x, y, z)``: ``z = 0`` is the ground and
``z = 1 .. n_layers`` are flight layers (a "2.5-D" airspace). Every building
has a height in layers and blocks only the layers at or below it, so drones
can overfly low buildings. *Pads* (delivery hubs and battery-swap stations)
are sites where drones can land; a landed drone is on the ground at ``z = 0``
and therefore outside the airspace. Take-off and landing are vertical moves
at a pad.

Temporary no-fly zones (NFZ) model events such as an emergency-services
operation: they are announced ``nfz_lead_time`` ticks before they activate,
so agents can plan around them (a NOTAM, in aviation terms). A zone closes a
range of layers (by default all of them).

The optional *heading rule* mirrors aviation's semicircular rule: east/west
flight uses odd layers and north/south flight even layers, so crossing
traffic is vertically separated. Climbing and descending are always allowed.
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass
from typing import Iterable

Site = tuple[int, int]            # a ground location: pad, customer, building footprint
Cell = tuple[int, int, int]       # a point of the airspace grid; z = 0 is the ground
INF = float("inf")

MOVES: tuple[Site, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))   # horizontal moves, in search order


def manhattan(a, b) -> int:
    """Horizontal grid distance between two sites or cells (altitude ignored)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def site_of(c) -> Site:
    return c[0], c[1]


def ground(c) -> Cell:
    """The ground cell below a site or cell."""
    return c[0], c[1], 0


def lift(c) -> Cell:
    """The airspace cell for ``c``: a site or a ground cell maps to layer 1."""
    if len(c) == 2 or c[2] == 0:
        return c[0], c[1], 1
    return c


@dataclass(frozen=True)
class NoFlyZone:
    zid: int
    rect: tuple[int, int, int, int]  # x0, y0, x1, y1 inclusive
    announce_t: int
    start_t: int
    end_t: int  # exclusive
    z0: int = 1                      # lowest closed layer
    z1: int | None = None            # highest closed layer (None = every layer)

    @property
    def cells(self) -> frozenset[Site]:
        """The zone's footprint (sites)."""
        x0, y0, x1, y1 = self.rect
        return frozenset((x, y) for x in range(x0, x1 + 1) for y in range(y0, y1 + 1))

    def contains(self, c) -> bool:
        """Is the site (any layer) or cell inside the zone?"""
        x0, y0, x1, y1 = self.rect
        if not (x0 <= c[0] <= x1 and y0 <= c[1] <= y1):
            return False
        if len(c) < 3:
            return True
        return self.z0 <= c[2] and (self.z1 is None or c[2] <= self.z1)

    def active(self, t: int) -> bool:
        return self.start_t <= t < self.end_t

    def layers(self, n_layers: int) -> tuple[int, int]:
        return self.z0, n_layers if self.z1 is None else min(self.z1, n_layers)


class GridWorld:
    def __init__(
        self,
        width: int,
        height: int,
        blocked: Iterable[Site],
        hubs: list[Site],
        stations: list[Site],
        customers: list[Site],
        nfzs: Iterable[NoFlyZone] = (),
        n_layers: int = 1,
        heights: dict[Site, int] | None = None,
        layer_rule: str = "free",
        customer_heights: dict[Site, int] | None = None,
    ):
        """``blocked`` are building footprints; ``heights`` gives their height in
        layers (a footprint without a height blocks every layer).

        ``customer_heights`` (optional) puts customers in buildings: a customer
        of height h >= 1 is a building h layers tall (it must also be in
        ``heights``) whose parcels are winched onto the roof from layer h + 1;
        h = 0 is a house, served from layer 1 as an open-ground customer is."""
        self.width = width
        self.height = height
        self.n_layers = n_layers
        # a single layer cannot separate headings: the rule degenerates to "free"
        self.layer_rule = layer_rule if n_layers > 1 else "free"
        hts = {tuple(c): n_layers for c in blocked}
        for c, h in (heights or {}).items():
            hts[tuple(c)] = max(1, min(n_layers, int(h)))
        self.heights: dict[Site, int] = hts
        self.blocked = frozenset(hts)
        self.hubs = list(hubs)
        self.stations = list(stations)
        self.customers = list(customers)
        self.customer_heights: dict[Site, int] = {tuple(c): h for c, h in (customer_heights or {}).items()}
        self.pads = frozenset(self.hubs) | frozenset(self.stations)
        self.nfzs: list[NoFlyZone] = sorted(nfzs, key=lambda z: z.announce_t)
        self._nbr_cache: dict[Cell, tuple[Cell, ...]] = {}
        self._dist_cache: dict[Cell, dict[Cell, int]] = {}
        for p in self.pads:
            if p in self.blocked:
                raise ValueError(f"pad {p} is inside a building")

    # ------------------------------------------------------------------ static
    def in_bounds(self, c) -> bool:
        return 0 <= c[0] < self.width and 0 <= c[1] < self.height

    def building_height(self, site) -> int:
        """Height of the building on ``site`` in layers (0 = open ground)."""
        return self.heights.get((site[0], site[1]), 0)

    def drop_cell(self, site) -> Cell:
        """Where a drone hovers to deliver to the customer at ``site``: the layer
        just above its roof (layer 1 for open ground or a house)."""
        return site[0], site[1], self.customer_heights.get((site[0], site[1]), 0) + 1

    def is_open(self, c) -> bool:
        """Can a drone fly in cell ``c``? (A site is read as its layer-1 cell.)"""
        z = c[2] if len(c) > 2 else 1
        return (self.in_bounds(c) and 1 <= z <= self.n_layers
                and self.heights.get((c[0], c[1]), 0) < z)

    def allows(self, dx: int, dy: int, z: int) -> bool:
        """Does the layer rule allow a horizontal move (dx, dy) on layer ``z``?"""
        if self.layer_rule == "free":
            return True
        return (z % 2 == 1) == (dx != 0)       # east/west on odd layers, north/south on even

    def neighbors(self, c: Cell) -> tuple[Cell, ...]:
        """Airspace cells reachable in one tick: 4 horizontal moves, then up, then down (cached).

        Descending from layer 1 to the ground is a landing, which only the
        planner performs (at a pad), so it is not a neighbour here.
        """
        nb = self._nbr_cache.get(c)
        if nb is None:
            x, y, z = c
            out = [(x + dx, y + dy, z) for dx, dy in MOVES
                   if self.allows(dx, dy, z) and self.is_open((x + dx, y + dy, z))]
            if z < self.n_layers and self.is_open((x, y, z + 1)):
                out.append((x, y, z + 1))
            if z > 1 and self.is_open((x, y, z - 1)):
                out.append((x, y, z - 1))
            nb = tuple(out)
            self._nbr_cache[c] = nb
        return nb

    def distance_map(self, goal) -> dict[Cell, int]:
        """Exact obstacle-aware distance in ticks from every airspace cell to ``goal`` (3-D BFS).

        ``goal`` is a cell, or a site meaning its layer-1 cell. Moves are the
        planner's moves (horizontal under the layer rule, climb, descend), all
        one tick, and the move graph is symmetric, so this is the exact
        static distance. Used as the (admissible and consistent) heuristic by
        the space-time planner and for fast energy estimates when bidding.
        Temporary NFZs and other drones are ignored here, which keeps the
        heuristic admissible: extra obstacles can only make true paths longer.
        """
        goal = lift(goal)
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

    def dist(self, a, b) -> float:
        """Flight distance in ticks from ``a`` to ``b`` (sites and ground cells count as layer 1)."""
        return self.distance_map(lift(b)).get(lift(a), INF)

    # ----------------------------------------------------------------- dynamic
    def known_zones(self, now: int) -> list[NoFlyZone]:
        """Zones an agent can know about at ``now`` that are not yet over."""
        return [z for z in self.nfzs if z.announce_t <= now and z.end_t > now]

    def zones_announced_at(self, t: int) -> list[NoFlyZone]:
        return [z for z in self.nfzs if z.announce_t == t]

    def in_active_nfz(self, c, t: int) -> bool:
        return any(z.active(t) and z.contains(c) for z in self.nfzs)

    # ----------------------------------------------------------------- display
    def ascii(self, marks: dict[Site, str] | None = None) -> str:
        """Top view; buildings show their height in layers (``#`` when the city is flat)."""
        marks = marks or {}
        rows = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                c = (x, y)
                if c in marks:
                    row.append(marks[c])
                elif c in self.blocked:
                    row.append("#" if self.n_layers == 1 else str(min(9, self.heights[c])))
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

def _largest_component(width: int, height: int, blocked: set[Site]) -> set[Site]:
    seen: set[Site] = set()
    best: set[Site] = set()
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


def _spread_sample(rng: random.Random, candidates: list[Site], k: int, taken: list[Site]) -> list[Site]:
    """Farthest-point sampling so pads are spread across the city."""
    chosen: list[Site] = []
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
    blocked: set[Site] = set()
    rects: list[tuple[int, int, int, int]] = []
    target = int(W * H * cfg.building_density)
    guard = 0
    while len(blocked) < target and guard < 10_000:
        guard += 1
        w, h = rng.randint(1, 3), rng.randint(1, 3)
        x0, y0 = rng.randrange(0, W - w + 1), rng.randrange(0, H - h + 1)
        rects.append((x0, y0, w, h))
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

    nfzs = [NoFlyZone(i, tuple(ev[3]), ev[0], ev[1], ev[2], *(ev[4] if len(ev) > 4 else ()))
            for i, ev in enumerate(cfg.nfz_events)]
    if cfg.auto_nfz and not nfzs:
        nfzs = _generate_nfzs(rng, cfg, W, H, pads)
    heights = _building_heights(cfg, rects, blocked)
    if not getattr(cfg, "customer_buildings", False):
        return GridWorld(W, H, blocked, hubs, stations, customers, nfzs,
                         n_layers=cfg.n_layers, heights=heights, layer_rule=cfg.layer_rule)
    cust = _customer_heights(cfg, customers)
    while True:
        world = GridWorld(W, H, blocked | {c for c, h in cust.items() if h}, hubs, stations, customers, nfzs,
                          n_layers=cfg.n_layers, heights={**heights, **{c: h for c, h in cust.items() if h}},
                          layer_rule=cfg.layer_rule, customer_heights=cust)
        # a customer building must not wall another customer in: serve such a customer as a house
        cut = [c for c in customers if cust[c] and world.dist(hubs[0], world.drop_cell(c)) == INF]
        if not cut:
            return world
        for c in cut:
            cust[c] = 0


def _customer_heights(cfg, customers: list[Site]) -> dict[Site, int]:
    """Height of each customer's building in layers: 0 = a house (parcel into the
    garden from layer 1), h >= 1 = a building whose roof is h layers up. Drawn
    from their own random stream, so nothing else in the city changes; capped
    one layer below the top so a drone can always hover above the roof."""
    rng = random.Random(cfg.seed * 7919 + 41)
    levels = list(range(min(len(cfg.customer_height_weights), cfg.n_layers)))
    weights = list(cfg.customer_height_weights)[: len(levels)]
    return {c: rng.choices(levels, weights)[0] for c in customers}


def _building_heights(cfg, rects: list[tuple[int, int, int, int]], blocked: set[Site]) -> dict[Site, int]:
    """Give every building block a height of 1..n_layers layers.

    Heights come from their own random stream, so the city's footprint (and
    everything else generated above) is identical for any number of layers.
    Low-rise is more common than high-rise: P(h) is proportional to
    n_layers - h + 1. Overlapping blocks keep the taller height; enclosed
    courtyards that were filled in count as 1-layer structures.
    """
    n = cfg.n_layers
    heights = {c: 1 for c in blocked}
    if n == 1:
        return heights
    rng = random.Random(cfg.seed * 7919 + 29)
    levels = list(range(1, n + 1))
    weights = [n - h + 1 for h in levels]
    for x0, y0, w, h in rects:
        level = rng.choices(levels, weights)[0]
        for x in range(x0, x0 + w):
            for y in range(y0, y0 + h):
                if (x, y) in heights:
                    heights[(x, y)] = max(heights[(x, y)], level)
    return heights


def _generate_nfzs(rng: random.Random, cfg, W: int, H: int, pads: set[Site]) -> list[NoFlyZone]:
    zones = []
    horizon = max(cfg.order_until, 200)
    starts = sorted(rng.sample(range(80, horizon - 40), 2)) if horizon > 140 else []
    alt = (1, cfg.nfz_ceiling) if cfg.nfz_ceiling is not None else ()
    for i, start in enumerate(starts):
        for _ in range(200):
            w, h = rng.randint(4, 6), rng.randint(3, 5)
            x0, y0 = rng.randrange(1, W - w - 1), rng.randrange(1, H - h - 1)
            rect = (x0, y0, x0 + w - 1, y0 + h - 1)
            # never close a pad (or its approach cells)
            if any(x0 - 1 <= p[0] <= rect[2] + 1 and y0 - 1 <= p[1] <= rect[3] + 1 for p in pads):
                continue
            dur = rng.randint(50, 80)
            zones.append(NoFlyZone(i, rect, start - cfg.nfz_lead_time, start, start + dur, *alt))
            break
    return zones
