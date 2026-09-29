"""Space-time A* for one drone, chained through a list of waypoints.

Search state is ``(cell, t, airborne)``. Actions per tick:

* airborne: move to a 4-neighbour, or hover in place
* on a pad (not airborne): wait on the ground (free of any conflict, costs no
  energy) or take off into the airspace above the pad

An air *segment* always ends by landing on a pad. Because a landed drone is out
of the airspace, every committed plan ends in a state that can never collide
with anyone - the property that makes cooperative planning safe without
unbounded "stay at goal" reservations.

The heuristic is the exact static BFS distance to the waypoint (+1 for a
pending take-off). Buildings are the only static obstacles, while other drones
and temporary no-fly zones only ever lengthen a path, so the heuristic is
admissible and A* returns the earliest-arrival path given everyone else's
reservations.
"""

from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field
from itertools import count

from .reservation import ReservationTable
from .world import Cell, GridWorld

HOVER_TIEBREAK = 0.01  # prefer flying / waiting on the ground over hovering


@dataclass(frozen=True)
class Waypoint:
    cell: Cell
    kind: str          # "drop" (hover for ``dwell`` ticks) or "land"
    dwell: int = 0


@dataclass
class PlanStep:
    t: int
    cell: Cell
    airborne: bool
    tag: str | None = None  # takeoff | drop_start | drop_done | land


@dataclass
class Plan:
    steps: list[PlanStep]
    waypoints: tuple[Waypoint, ...] = field(default_factory=tuple)

    @property
    def start_t(self) -> int:
        return self.steps[0].t

    @property
    def end_t(self) -> int:
        return self.steps[-1].t

    def step_at(self, t: int) -> PlanStep | None:
        i = t - self.start_t
        if 0 <= i < len(self.steps):
            return self.steps[i]
        return None

    def remaining(self, t: int) -> list[PlanStep]:
        i = max(0, t - self.start_t)
        return self.steps[i:]

    def is_contiguous(self) -> bool:
        return all(b.t == a.t + 1 for a, b in zip(self.steps, self.steps[1:]))

    def airborne_states(self):
        return ((s.cell, s.t, s.airborne) for s in self.steps)

    def path_cells(self, from_t: int) -> list[Cell]:
        return [s.cell for s in self.remaining(from_t)]


@dataclass
class PlannerStats:
    searches: int = 0
    failures: int = 0
    expansions: int = 0
    seconds: float = 0.0


class SpaceTimePlanner:
    def __init__(self, world: GridWorld, reservations: ReservationTable, max_expansions: int = 40000):
        self.world = world
        self.res = reservations
        self.max_expansions = max_expansions
        self.stats = PlannerStats()

    # ---------------------------------------------------------------- public
    def plan(
        self,
        agent: int,
        start: Cell,
        start_t: int,
        airborne: bool,
        waypoints: list[Waypoint],
        now: int,
        ignore: frozenset = frozenset(),
        extra_blocked: set | None = None,
    ) -> Plan | None:
        """Plan through ``waypoints`` (the last must be a landing).

        Returns ``None`` if any segment cannot be found within the search
        budget; the caller then holds position (in the air) or stays on the
        ground and retries later.
        """
        assert waypoints and waypoints[-1].kind == "land", "air segments must end with a landing"
        t_begin = time.perf_counter()
        zones = self.world.known_zones(now)
        steps = [PlanStep(start_t, start, airborne)]
        cur, t, air = start, start_t, airborne
        try:
            for wp in waypoints:
                seg = self._search(agent, cur, t, air, wp, zones, ignore, extra_blocked)
                if seg is None:
                    self.stats.failures += 1
                    return None
                for c, st, sa in seg[1:]:
                    tag = "takeoff" if sa and not steps[-1].airborne else None
                    steps.append(PlanStep(st, c, sa, tag))
                if wp.kind == "drop":
                    t_arr = steps[-1].t
                    steps[-1].tag = "drop_start"
                    for k in range(1, wp.dwell + 1):
                        steps.append(PlanStep(t_arr + k, wp.cell, True))
                    steps[-1].tag = "drop_done"
                    cur, t, air = wp.cell, t_arr + wp.dwell, True
                else:
                    steps[-1].tag = "land"
                    cur, t, air = wp.cell, steps[-1].t, False
            return Plan(steps, tuple(waypoints))
        finally:
            self.stats.seconds += time.perf_counter() - t_begin

    # --------------------------------------------------------------- internal
    def _search(self, agent, start, t0, airborne, wp: Waypoint, zones, ignore, extra_blocked):
        self.stats.searches += 1
        world, res = self.world, self.res
        goal = wp.cell
        dm = world.distance_map(goal)
        if start not in dm:
            return None
        dwell = wp.dwell if wp.kind == "drop" else 0

        def in_zone(c: Cell, t: int) -> bool:
            for z in zones:
                if z.start_t <= t < z.end_t and z.contains(c):
                    return True
            return False

        # Earliest tick the goal itself can be occupied (a customer inside a
        # no-fly zone must wait for it to lift). Folding this into the
        # heuristic, h = max(distance, open_t - t), keeps it admissible and
        # consistent while stopping A* from flooding space-time with hovering.
        open_t = t0
        goal_zones = [z for z in zones if z.contains(goal)]
        while any(in_zone(goal, open_t + k) for k in range(dwell + 1)) and goal_zones:
            open_t = max(z.end_t for z in goal_zones if z.start_t <= open_t + dwell and z.end_t > open_t)
        horizon = max(t0 + 3 * dm[start] + 80, open_t + dm[start] + 40)

        def blocked(c: Cell, t: int) -> bool:
            return in_zone(c, t) or (extra_blocked is not None and (c, t) in extra_blocked)

        def goal_ok(t: int) -> bool:
            if dwell == 0:
                return True
            for k in range(1, dwell + 1):
                if blocked(goal, t + k) or not res.vertex_free(goal, t + k, agent, ignore):
                    return False
            return True

        tie = count()
        s0 = (start, t0, airborne)
        h0 = max(dm[start] + (0 if airborne else 1), open_t - t0)
        open_heap = [(h0, -t0, next(tie), 0.0, s0)]
        parent: dict = {s0: None}
        parent_g: dict = {s0: 0.0}
        closed: set = set()
        expansions = 0
        while open_heap:
            f, _, _, g, state = heapq.heappop(open_heap)
            if state in closed:
                continue
            closed.add(state)
            cell, t, air = state
            if air and cell == goal and goal_ok(t):
                self.stats.expansions += expansions
                path = []
                while state is not None:
                    path.append(state)
                    state = parent[state]
                path.reverse()
                return path
            expansions += 1
            if expansions > self.max_expansions or t >= horizon:
                if expansions > self.max_expansions:
                    break
                continue
            nt = t + 1
            if not air:
                succ = [((cell, nt, False), 1.0)]
                if not blocked(cell, nt) and res.vertex_free(cell, nt, agent, ignore):
                    succ.append(((cell, nt, True), 1.0))
            else:
                succ = []
                for n in world.neighbors(cell) + (cell,):
                    if blocked(n, nt) or not res.move_free(cell, n, t, agent, ignore):
                        continue
                    succ.append(((n, nt, True), 1.0 + (HOVER_TIEBREAK if n == cell else 0.0)))
            for nxt, cost in succ:
                if nxt in closed:
                    continue
                ng = g + cost
                h = dm.get(nxt[0])
                if h is None:
                    continue
                if not nxt[2]:
                    h += 1
                if open_t > nt:
                    h = max(h, open_t - nt)
                if ng < parent_g.get(nxt, float("inf")):
                    parent[nxt] = state
                    parent_g[nxt] = ng
                    heapq.heappush(open_heap, (ng + h, -nt, next(tie), ng, nxt))
        self.stats.expansions += expansions
        return None
