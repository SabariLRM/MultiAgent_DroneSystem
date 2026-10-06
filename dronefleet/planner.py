"""Space-time A* for one drone, chained through a list of waypoints.

Search state is ``(cell, t)`` with ``cell = (x, y, z)``; ``z = 0`` means the
drone is on the ground (only ever at a pad), ``z >= 1`` that it is airborne on
that flight layer. Actions per tick:

* airborne: move to a horizontal neighbour (subject to the layer rule), climb
  one layer, descend one layer (not below layer 1), or hover in place
* on a pad: wait on the ground (free of any conflict, costs no energy) or take
  off vertically to layer 1 above the pad

An air *segment* always ends by landing on a pad: the drone arrives at layer 1
above the pad and touches down at the end of that tick. Because a landed drone
is out of the airspace, every committed plan ends in a state that can never
collide with anyone - the property that makes cooperative planning safe
without unbounded "stay at goal" reservations. Parcels are lowered from
layer 1 above the customer.

The heuristic is the exact static 3-D BFS distance to the waypoint (+1 for a
pending take-off). Buildings (up to their height) are the only static
obstacles, while other drones and temporary no-fly zones only ever lengthen a
path, so the heuristic is admissible and consistent, and A* returns the
earliest-arrival path given everyone else's reservations.
"""

from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field
from itertools import count

from .reservation import ReservationTable
from .world import Cell, GridWorld, ground, lift

HOVER_TIEBREAK = 0.01     # prefer flying / waiting on the ground over hovering ...
VERTICAL_TIEBREAK = 0.02  # ... and hovering over a climb or descent that arrives no sooner
ZONE_ESCAPE_COST = 1000.0 # per tick inside a no-fly zone the drone is already caught in


@dataclass(frozen=True)
class Waypoint:
    cell: tuple        # a site (x, y) means its layer-1 cell
    kind: str          # "drop" (hover for ``dwell`` ticks) or "land"
    dwell: int = 0


@dataclass
class PlanStep:
    t: int
    cell: Cell
    airborne: bool
    tag: str | None = None  # takeoff | drop_start | drop_done | land_start (continuous) | land


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
    def __init__(self, world: GridWorld, reservations: ReservationTable, max_expansions: int = 40000,
                 land_dwell: int = 0, cruise_layer: int = 0, cruise_penalty: float = 0.0):
        """``max_expansions`` is the search budget per flight layer (the airspace grows with the layers).

        ``land_dwell`` (continuous flight only; 0 in grid mode) keeps the drone
        over the pad for that many extra ticks after it arrives, so the pad's
        layer-1 cell stays reserved while the vertical descent passes through
        it. The arrival step is tagged ``land_start``.

        ``cruise_layer`` (0 = off) makes drones prefer to cruise high: every
        horizontal move below that layer costs ``cruise_penalty`` extra per
        layer below it. On a longer trip the planner then climbs over the
        buildings and descends near the destination, while a short hop stays
        low. Costs only grow, so the heuristic stays admissible and
        consistent; A* still never plans a conflict, it just trades arrival
        time for altitude.
        """
        self.world = world
        self.res = reservations
        self.max_expansions = max_expansions * world.n_layers
        self.land_dwell = land_dwell
        self.cruise_layer = min(cruise_layer, world.n_layers)
        self.cruise_penalty = cruise_penalty
        self.stats = PlannerStats()

    # ---------------------------------------------------------------- public
    def plan(
        self,
        agent: int,
        start,
        start_t: int,
        airborne: bool,
        waypoints: list[Waypoint],
        now: int,
        ignore: frozenset = frozenset(),
        extra_blocked: set | None = None,
    ) -> Plan | None:
        """Plan from ``start`` through ``waypoints`` (the last must be a landing).

        ``start`` is a cell, or a site read as layer 1 (``airborne``) or the
        ground. Returns ``None`` if any segment cannot be found within the
        search budget; the caller then holds position (in the air) or stays on
        the ground and retries later.
        """
        assert waypoints and waypoints[-1].kind == "land", "air segments must end with a landing"
        t_begin = time.perf_counter()
        if len(start) == 2:
            start = lift(start) if airborne else ground(start)
        zones = self.world.known_zones(now)
        steps = [PlanStep(start_t, start, start[2] > 0)]
        cur, t = start, start_t
        try:
            for wp in waypoints:
                goal = lift(wp.cell)
                dwell = wp.dwell if wp.kind == "drop" else self.land_dwell
                seg = self._search(agent, cur, t, goal, dwell, zones, ignore, extra_blocked)
                if seg is None:
                    self.stats.failures += 1
                    return None
                for c, st in seg[1:]:
                    sa = c[2] > 0
                    tag = "takeoff" if sa and not steps[-1].airborne else None
                    steps.append(PlanStep(st, c, sa, tag))
                if wp.kind == "drop":
                    t_arr = steps[-1].t
                    steps[-1].tag = "drop_start"
                    for k in range(1, dwell + 1):
                        steps.append(PlanStep(t_arr + k, goal, True))
                    steps[-1].tag = "drop_done"
                    cur, t = goal, t_arr + dwell
                else:
                    if dwell:
                        t_arr = steps[-1].t
                        steps[-1].tag = "land_start"
                        for k in range(1, dwell + 1):
                            steps.append(PlanStep(t_arr + k, goal, True))
                    steps[-1].tag = "land"
                    cur, t = ground(goal), steps[-1].t
            return Plan(steps, tuple(waypoints))
        finally:
            self.stats.seconds += time.perf_counter() - t_begin

    # --------------------------------------------------------------- internal
    def _search(self, agent, start: Cell, t0: int, goal: Cell, dwell: int, zones, ignore, extra_blocked):
        self.stats.searches += 1
        world, res = self.world, self.res
        dm = world.distance_map(goal)
        start_air = lift(start)
        if start_air not in dm:
            return None
        airborne = start[2] > 0

        def in_zone(c, t: int, among=zones) -> bool:
            for z in among:
                if z.start_t <= t < z.end_t and z.contains(c):
                    return True
            return False

        # A drone can be caught inside an active zone - typically held there by
        # traffic when the zone started. Forbidding every zone cell would leave
        # it no move at all, so it would hover in the zone until the zone lifts
        # (and could run its battery flat). Instead the cells of the zones it is
        # in cost ZONE_ESCAPE_COST per tick: A* leaves by the quickest route and
        # never lingers. Costs only grow, so the heuristic stays admissible and
        # consistent; a drone outside every active zone plans exactly as before.
        escape = [z for z in zones if airborne and z.active(t0) and z.contains(start)]
        hard = [z for z in zones if z not in escape] if escape else zones

        # Earliest tick the goal itself can be occupied (a customer inside a
        # no-fly zone must wait for it to lift). Folding this into the
        # heuristic, h = max(distance, open_t - t), keeps it admissible and
        # consistent while stopping A* from flooding space-time with hovering.
        open_t = t0
        goal_zones = [z for z in zones if z.contains(goal)]
        while any(in_zone(goal, open_t + k) for k in range(dwell + 1)) and goal_zones:
            open_t = max(z.end_t for z in goal_zones if z.start_t <= open_t + dwell and z.end_t > open_t)
        horizon = max(t0 + 3 * dm[start_air] + 80, open_t + dm[start_air] + 40)

        def blocked(c, t: int) -> bool:
            return in_zone(c, t, hard) or (extra_blocked is not None and (c, t) in extra_blocked)

        def goal_ok(t: int) -> bool:
            if escape and in_zone(goal, t, escape):
                return False
            if dwell == 0:
                return True
            for k in range(1, dwell + 1):
                if blocked(goal, t + k) or (escape and in_zone(goal, t + k, escape)) \
                        or not res.vertex_free(goal, t + k, agent, ignore):
                    return False
            return True

        tie = count()
        s0 = (start, t0)
        h0 = max(dm[start_air] + (0 if airborne else 1), open_t - t0)
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
            cell, t = state
            if cell == goal and goal_ok(t):
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
            x, y, z = cell
            if z == 0:
                succ = [((cell, nt), 1.0)]
                up = (x, y, 1)
                if not blocked(up, nt) and res.vertex_free(up, nt, agent, ignore):
                    succ.append(((up, nt), 1.0))
            else:
                succ = []
                for n in world.neighbors(cell) + (cell,):
                    if blocked(n, nt) or not res.move_free(cell, n, t, agent, ignore):
                        continue
                    if n == cell:
                        cost = 1.0 + HOVER_TIEBREAK
                    elif n[2] != z:
                        cost = 1.0 + VERTICAL_TIEBREAK
                    else:
                        cost = 1.0
                        if z < self.cruise_layer:
                            cost += self.cruise_penalty * (self.cruise_layer - z)
                    if escape and in_zone(n, nt, escape):
                        cost += ZONE_ESCAPE_COST
                    succ.append(((n, nt), cost))
            for nxt, cost in succ:
                if nxt in closed:
                    continue
                ng = g + cost
                c = nxt[0]
                if c[2] == 0:
                    h = dm.get((c[0], c[1], 1))
                    if h is None:
                        continue
                    h += 1
                else:
                    h = dm.get(c)
                    if h is None:
                        continue
                if open_t > nt:
                    h = max(h, open_t - nt)
                if ng < parent_g.get(nxt, float("inf")):
                    parent[nxt] = state
                    parent_g[nxt] = ng
                    heapq.heappush(open_heap, (ng + h, -nt, next(tie), ng, nxt))
        self.stats.expansions += expansions
        return None
