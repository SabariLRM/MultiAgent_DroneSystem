"""Continuous 3-D flight: the tactical layer and the physics (``SimConfig.motion = "continuous"``).

The strategic layer (space-time A* + reservation table) still decides *where
each drone should be at every 10-second tick*. This module flies those 4-D
waypoints in metres and seconds (see ``docs/continuous_design.md``):

* :func:`build_reference` turns a committed :class:`~dronefleet.planner.Plan`
  into a time-parameterised trajectory, optionally smoothed by line-of-sight
  shortcuts that stay inside the reserved space-time tube and clear of
  buildings. Take-off, the parcel winch and landing stay vertical.
* :class:`FlightLayer` advances every drone in ``physics_dt`` sub-steps: a
  path follower gives the preferred velocity, 3-D ORCA (:mod:`dronefleet.orca`)
  adjusts it against neighbours found through a spatial hash, and a point-mass
  model with acceleration limits, wind (:mod:`dronefleet.wind`) and a power
  model (:mod:`dronefleet.power`) moves the drone and drains its battery.
* At each tick boundary :meth:`FlightLayer.observe` reports a grid cell back
  to the drone agent, which then repairs, re-plans or holds exactly as it
  does after a wind gust in grid mode. Drones that make no progress are handed
  back to the strategic layer (hold -> yield -> escalation).
* A separation monitor counts collisions (3-D distance < 2 x radius), losses
  of separation (inside the ``sep_h`` x ``sep_v`` bubble) and the minimum
  separation, and the trace sampler records positions for the replay viewers.

Axes: x east and y along the grid rows (cell ``(x, y)`` is centred at
``((x + 0.5) * cell_m, (y + 0.5) * cell_m)``), z up (layer ``z`` flies at
``z * layer_m``).
"""

from __future__ import annotations

import bisect
import math

from .orca import orca_plane, solve
from .power import PowerModel
from .world import ground

# what a stretch of the reference trajectory asks the drone to do
GROUND, TAKEOFF, FLY, WINCH, LAND, HOLD = range(6)
MODE_NAMES = ("ground", "takeoff", "fly", "winch", "land", "hold")

KEEP_RIGHT = math.radians(8.0)   # symmetry breaking for head-on ORCA encounters ...
UNBLOCK = math.radians(75.0)     # ... and, stronger, for a drone that is stuck in front of others
INTERVENTION = 0.5               # m/s: ORCA "intervened" if it moved the velocity this much
BUILDING_RANGE = 60.0            # m: buildings closer than this constrain ORCA's velocity ...
BUILDING_MARGIN = 8.0            # ... so that the drone stays this far from the wall ...
BUILDING_HORIZON = 4.0           # ... for at least this many seconds


# ============================================================================
# reference trajectories
# ============================================================================

class Reference:
    """A plan as a 4-D trajectory: nodes ``(t, x, y, z)`` and the mode of each segment."""

    __slots__ = ("t", "x", "y", "z", "mode", "after", "stop_d", "k", "goal")

    def __init__(self, t, x, y, z, mode, after, goal):
        self.t, self.x, self.y, self.z = t, x, y, z
        self.mode = mode          # mode[k] for the segment node k -> node k + 1
        self.after = after        # what to do after the last node (LAND or HOLD)
        self.goal = goal          # (x, y, z) of the next stop that matters (drop point or pad)
        self.k = 0
        n = len(t)
        stop_d = [0.0] * n
        for i in range(n - 2, -1, -1):
            dx, dy, dz = x[i + 1] - x[i], y[i + 1] - y[i], z[i + 1] - z[i]
            moving = mode[i] == FLY and (dx or dy or dz)
            stop_d[i] = stop_d[i + 1] + math.sqrt(dx * dx + dy * dy + dz * dz) if moving else 0.0
        self.stop_d = stop_d      # path length from node i to the next node where the reference stops

    def seg(self, now: float) -> int:
        t = self.t
        k = self.k
        if now < t[k]:
            k = max(0, bisect.bisect_right(t, now) - 1)
        else:
            n = len(t)
            while k + 1 < n and t[k + 1] <= now:
                k += 1
        self.k = k
        return k

    def at(self, now: float):
        """(x, y, z, vx, vy, vz, mode, k) of the reference at time ``now``."""
        k = self.seg(now)
        t = self.t
        if k >= len(t) - 1 or now < t[0]:
            if now < t[0] and len(t) > 1:
                return self.x[0], self.y[0], self.z[0], 0.0, 0.0, 0.0, self.mode[0], 0
            return self.x[-1], self.y[-1], self.z[-1], 0.0, 0.0, 0.0, self.after, len(t) - 1
        span = t[k + 1] - t[k]
        vx = (self.x[k + 1] - self.x[k]) / span
        vy = (self.y[k + 1] - self.y[k]) / span
        vz = (self.z[k + 1] - self.z[k]) / span
        e = now - t[k]
        return self.x[k] + vx * e, self.y[k] + vy * e, self.z[k] + vz * e, vx, vy, vz, self.mode[k], k


def centre(cell, cfg) -> tuple[float, float, float]:
    """Metres of a cell centre (a ground cell is on the ground)."""
    return (cell[0] + 0.5) * cfg.cell_m, (cell[1] + 0.5) * cfg.cell_m, cell[2] * cfg.layer_m


def build_reference(plan, cfg, world, start=None) -> Reference:
    """The 4-D trajectory of ``plan``; ``start`` (metres) replaces its first point if given."""
    steps = plan.steps
    n = len(steps)
    drop_s = drop_e = land_s = -1
    for i, s in enumerate(steps):
        if s.tag == "drop_start":
            drop_s = i
        elif s.tag == "drop_done":
            drop_e = i
        elif s.tag == "land_start":
            land_s = i
    last_land = steps[-1].tag == "land"
    if last_land and land_s < 0:
        land_s = n - 1
    P = [centre(s.cell, cfg) for s in steps]
    T = [s.t * cfg.tick_s for s in steps]
    modes = []
    for i in range(n - 1):
        a, b = steps[i], steps[i + 1]
        if not b.airborne:
            m = GROUND
        elif not a.airborne:
            m = TAKEOFF
        elif 0 <= drop_s <= i < drop_e:
            m = WINCH
        elif 0 <= land_s <= i:
            m = LAND
        else:
            m = FLY
        modes.append(m)
    if start is not None and steps[0].airborne:
        P[0] = start
    goal_i = drop_s if (drop_s >= 0 and drop_e >= 0) else n - 1
    goal = P[goal_i]
    if cfg.smoothing and n > 2:
        P, T, modes = _smooth(P, T, modes, cfg, world)
    return Reference(T, [p[0] for p in P], [p[1] for p in P], [p[2] for p in P], modes,
                     LAND if last_land else HOLD, goal)


def _smooth(P, T, modes, cfg, world):
    """Greedy line-of-sight shortcutting over runs of FLY segments (see module doc)."""
    out_P, out_T, out_M = [P[0]], [T[0]], []
    n = len(P)
    i = 0
    tol2 = (cfg.smooth_tol * cfg.cell_m) ** 2
    vtol = 0.34 * cfg.layer_m
    while i < n - 1:
        if modes[i] != FLY:
            out_P.append(P[i + 1]); out_T.append(T[i + 1]); out_M.append(modes[i])
            i += 1
            continue
        j = i
        while j < n - 1 and modes[j] == FLY:
            j += 1
        # dense raw points: nodes i..j and the midpoints of their segments
        pts, tms = [P[i]], [T[i]]
        for k in range(i, j):
            a, b = P[k], P[k + 1]
            pts.append(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2))
            tms.append((T[k] + T[k + 1]) / 2)
            pts.append(b)
            tms.append(T[k + 1])
        a = 0
        m = len(pts)
        while a < m - 1:
            b = a + 1
            while b + 1 < m and _shortcut_ok(pts, tms, a, b + 1, tol2, vtol, cfg, world):
                b += 1
            out_P.append(pts[b]); out_T.append(tms[b]); out_M.append(FLY)
            a = b
        i = j
    # merge collinear FLY runs produced by midpoints on straight legs
    return _merge(out_P, out_T, out_M)


def _merge(P, T, M):
    rP, rT, rM = [P[0]], [T[0]], []
    for k in range(1, len(P)):
        if (rM and rM[-1] == FLY and M[k - 1] == FLY and k < len(P) and len(rP) >= 2):
            a, b, c = rP[-2], rP[-1], P[k]
            ta, tb, tc = rT[-2], rT[-1], T[k]
            if tc > ta:
                f = (tb - ta) / (tc - ta)
                q = (a[0] + (c[0] - a[0]) * f, a[1] + (c[1] - a[1]) * f, a[2] + (c[2] - a[2]) * f)
                if abs(q[0] - b[0]) < 1e-6 and abs(q[1] - b[1]) < 1e-6 and abs(q[2] - b[2]) < 1e-6:
                    rP[-1], rT[-1] = c, tc          # b lies on a -> c at the right time: drop it
                    continue
        rP.append(P[k]); rT.append(T[k]); rM.append(M[k - 1])
    return rP, rT, rM


def _shortcut_ok(pts, tms, a, b, tol2, vtol, cfg, world) -> bool:
    ax, ay, az = pts[a]
    bx, by, bz = pts[b]
    ta, tb = tms[a], tms[b]
    dur = tb - ta
    if dur <= 0:
        return False
    dz = bz - az
    if dz > cfg.max_climb * dur + 1e-6 or -dz > cfg.max_descent * dur + 1e-6:
        return False
    for k in range(a + 1, b):
        f = (tms[k] - ta) / dur
        p = pts[k]
        qx, qy, qz = ax + (bx - ax) * f - p[0], ay + (by - ay) * f - p[1], az + dz * f - p[2]
        if qx * qx + qy * qy > tol2 or abs(qz) > vtol:
            return False
    return segment_clear(world, cfg, ax, ay, az, bx, by, bz)


def segment_clear(world, cfg, x1, y1, z1, x2, y2, z2) -> bool:
    """Does the 3-D segment keep ``building_margin_m`` from every building box (and stay in the city)?"""
    cm, m = cfg.cell_m, cfg.building_margin_m
    lo_x, hi_x = min(x1, x2) - m, max(x1, x2) + m
    lo_y, hi_y = min(y1, y2) - m, max(y1, y2) + m
    if lo_x < 0 or lo_y < 0 or hi_x > world.width * cm or hi_y > world.height * cm:
        return False
    heights = world.heights
    zlow = min(z1, z2)
    dx, dy = x2 - x1, y2 - y1
    for cx in range(int(lo_x // cm), int(hi_x // cm) + 1):
        for cy in range(int(lo_y // cm), int(hi_y // cm) + 1):
            h = heights.get((cx, cy))
            if not h:
                continue
            roof = (h + 0.5) * cfg.layer_m + 5.0
            if zlow > roof:
                continue
            # clip the segment against the building box grown by the margin (slab test)
            s0, s1 = 0.0, 1.0
            ok = True
            for p, d, lo, hi in ((x1, dx, cx * cm - m, (cx + 1) * cm + m), (y1, dy, cy * cm - m, (cy + 1) * cm + m)):
                if abs(d) < 1e-12:
                    if p < lo or p > hi:
                        ok = False
                        break
                    continue
                ta, tb = (lo - p) / d, (hi - p) / d
                if ta > tb:
                    ta, tb = tb, ta
                s0, s1 = max(s0, ta), min(s1, tb)
                if s0 > s1:
                    ok = False
                    break
            if ok and min(z1 + (z2 - z1) * s0, z1 + (z2 - z1) * s1) <= roof:
                return False
    return True


# ============================================================================
# bodies and the flight layer
# ============================================================================

class Body:
    """Physical state of one drone plus its tactical bookkeeping."""

    __slots__ = ("i", "drone", "x", "y", "z", "vx", "vy", "vz", "ground", "dead", "wex", "wey",
                 "ref", "plan", "hold", "mode", "coop", "px", "py", "pz", "cx", "cy", "cz",
                 "energy", "dist_h", "nb", "orca_on", "orca_t0", "orca_tick", "q", "track",
                 "stall_best", "stall_t", "stalls", "inside", "pad", "spot")

    def __init__(self, i, drone, x, y):
        self.i, self.drone = i, drone
        self.x, self.y, self.z = x, y, 0.0
        self.vx = self.vy = self.vz = 0.0
        self.ground, self.dead = True, False
        self.wex = self.wey = 0.0
        self.ref = None
        self.plan = None
        self.hold = None
        self.mode, self.coop = GROUND, True
        self.px = self.py = self.pz = self.cx = self.cy = self.cz = 0.0
        self.energy = self.dist_h = 0.0
        self.nb = []
        self.orca_on, self.orca_t0, self.orca_tick = False, 0.0, False
        self.q = (0, 0, 0)
        self.track: list[int] = []
        self.stall_best, self.stall_t, self.stalls = math.inf, 0.0, 0
        self.inside = False
        self.pad = None           # (x, y): finish a landing here (the agent already counts it as landed)
        self.spot = None          # (pad site, index) of the touchdown spot whose column it is using


class FlightLayer:
    def __init__(self, sim):
        self.sim = sim
        self.cfg = cfg = sim.cfg
        self.world = sim.world
        self.wind = sim.wind
        self.pm = PowerModel(cfg)
        self.dt = cfg.physics_dt
        self.n_sub = int(round(cfg.tick_s / cfg.physics_dt))
        self.trace_every = int(round(cfg.trace_dt / cfg.physics_dt))
        self.orca = cfg.tactical == "orca"
        self.s_v = cfg.sep_h / cfg.sep_v                 # vertical scale that makes the bubble round
        self.R = cfg.orca_margin * cfg.sep_h
        self.bucket = cfg.sense_radius_m
        self.vplanes = [(0.0, 0.0, cfg.max_climb * self.s_v, 0.0, 0.0, -1.0),
                        (0.0, 0.0, -cfg.max_descent * self.s_v, 0.0, 0.0, 1.0)]
        self.bodies = [Body(i, d, *centre(ground(d.pos), cfg)[:2]) for i, d in enumerate(sim.drones)]
        # vertiports: every hub and station has pad_spots touchdown spots in its cell
        o = cfg.pad_spot_offset_m
        corners = [(-o, -o), (o, o), (o, -o), (-o, o)]
        offs = [(0.0, 0.0)] if cfg.pad_spots == 1 else corners[:cfg.pad_spots]
        self.spots = {p: [((p[0] + 0.5) * cfg.cell_m + dx, (p[1] + 0.5) * cfg.cell_m + dy) for dx, dy in offs]
                      for p in sorted(sim.world.pads)}
        self.spot_user: dict = {}
        self._hf: dict[float, float] = {}
        self._grid: dict = {}
        # safety and performance measures
        self.collisions = 0
        self.los_events = 0
        self.los_time = 0.0
        self.min_sep = math.inf
        self.track_sum = self.track_time = 0.0
        self.track_hist = [0.0] * 201                  # seconds per 1 m bin (last bin: >= 200 m)
        self.orca_events = 0
        self.flight_time = 0.0
        self.stall_replans = 0
        self.pad_wait = 0.0
        self.building_intrusions = 0
        self._los_open: dict[tuple, list] = {}
        self._touching: set = set()
        # replay trace
        self.samples = 0
        self.los_log: list[list] = []
        self.orca_log: list[list] = []
        self.coll_log: list[list] = []
        self.wind_log: list[int] = []
        self.events: list[str] = []
        self.orca_drones: set[int] = set()             # ORCA steered these drones during the last tick
        self.record = cfg.record_trace
        if self.record:
            self._sample(0.0)

    # ------------------------------------------------------------------ helpers
    def _hover_factor(self, payload: float) -> float:
        hf = self._hf.get(payload)
        if hf is None:
            hf = self._hf[payload] = self.pm.hover_factor(payload)
        return hf

    def _rebuild(self, b: Body) -> None:
        d = b.drone
        b.plan = d.plan
        b.pad = None
        if d.plan is None:
            b.ref = None
            if not b.ground:
                if d.pos[2] == 0:
                    # the agent's plan has ended with the landing but the descent is not
                    # finished yet: keep descending onto the pad
                    b.pad = centre(d.pos, self.cfg)[:2]
                else:
                    b.hold = (b.x, b.y, max(1, d.pos[2]) * self.cfg.layer_m)
            return
        start = None
        if not b.ground and d.plan.steps[0].airborne:
            cx, cy, cz = centre(d.plan.steps[0].cell, self.cfg)
            if math.hypot(b.x - cx, b.y - cy) < 1.5 * self.cfg.cell_m and abs(b.z - cz) < 20.0:
                start = (b.x, b.y, b.z)
        b.ref = build_reference(d.plan, self.cfg, self.world, start)
        b.stall_best, b.stall_t = math.inf, 0.0

    # ------------------------------------------------------------------ one tick
    def advance(self, t: int) -> None:
        """Integrate the physics from tick ``t`` to ``t + 1``."""
        self.orca_drones = set()
        for b in self.bodies:
            b.orca_tick = False
        base = t * self.cfg.tick_s
        for k in range(self.n_sub):
            now = base + k * self.dt
            self._substep(now)
            if self.record and (k + 1) % self.trace_every == 0:
                self._sample(now + self.dt)
        self.orca_drones = {b.i for b in self.bodies if b.orca_tick}

    def _substep(self, now: float) -> None:
        cfg = self.cfg
        dt = self.dt
        bodies = self.bodies
        for b in bodies:
            if not b.dead and b.drone.plan is not b.plan:
                self._rebuild(b)

        # spatial hash of the airborne drones
        B = self.bucket
        grid: dict = {}
        air = []
        for b in bodies:
            b.nb = []
            if b.dead or b.ground:
                continue
            air.append(b)
            key = (int(b.x // B), int(b.y // B))
            lst = grid.get(key)
            if lst is None:
                grid[key] = [b]
            else:
                lst.append(b)
        self._grid = grid

        # neighbour pairs (3 x 3 buckets) -> separation monitor and neighbour lists
        sv = self.s_v
        sense2 = B * B
        seen: set = set()
        for (gx, gy), lst in grid.items():
            for ox in (-1, 0, 1):
                for oy in (-1, 0, 1):
                    other = grid.get((gx + ox, gy + oy))
                    if other is None:
                        continue
                    for bi in lst:
                        for bj in other:
                            if bj.i <= bi.i:
                                continue
                            dx, dy, dz = bj.x - bi.x, bj.y - bi.y, bj.z - bi.z
                            dh2 = dx * dx + dy * dy
                            if dh2 > sense2:
                                continue
                            ds2 = dh2 + dz * dz * sv * sv
                            if ds2 > sense2:
                                continue
                            self._monitor(bi, bj, dh2, dz, now, seen)
                            bi.nb.append((ds2, bj))
                            bj.nb.append((ds2, bi))
        if len(seen) != len(self._los_open):
            for key in [k for k in self._los_open if k not in seen]:
                self._close_los(key, now)
        if self._touching:
            self._touching &= seen

        # preferred velocities (path following) and vertiport rules
        for b in bodies:
            if not b.dead:
                self._control(b, now)

        # tactical avoidance (drones that just lifted off have no neighbours listed yet)
        air = [b for b in bodies if not b.dead and not b.ground]
        if self.orca:
            for b in air:
                if b.coop and b.nb:
                    self._orca(b, now)
                else:
                    b.cx, b.cy, b.cz = b.px, b.py, b.pz
                    self._orca_state(b, False, now)
        else:
            for b in air:
                b.cx, b.cy, b.cz = b.px, b.py, b.pz

        # dynamics, wind and energy
        amax = cfg.max_accel
        D = cfg.wind_response
        vair = cfg.max_airspeed
        kw = dt / cfg.wind_estimator_s
        wind = self.wind
        heights = self.world.heights
        cm, lm = cfg.cell_m, cfg.layer_m
        for b in bodies:
            if b.dead or b.ground:
                continue
            ax, ay, az = (b.cx - b.vx) / dt, (b.cy - b.vy) / dt, (b.cz - b.vz) / dt
            a = math.sqrt(ax * ax + ay * ay + az * az)
            if a > amax:
                f = amax / a
                ax, ay, az = ax * f, ay * f, az * f
            vx, vy, vz = b.vx + ax * dt, b.vy + ay * dt, b.vz + az * dt
            wx, wy = wind.at(b.x, b.y, b.z, now)
            vx += D * (wx - b.wex) * dt
            vy += D * (wy - b.wey) * dt
            rx, ry = vx - wx, vy - wy
            asp = math.sqrt(rx * rx + ry * ry)
            if asp > vair:
                f = vair / asp
                vx, vy = wx + rx * f, wy + ry * f
            b.x += vx * dt
            b.y += vy * dt
            b.z += vz * dt
            b.vx, b.vy, b.vz = vx, vy, vz
            b.wex += (wx - b.wex) * kw
            b.wey += (wy - b.wey) * kw
            rx, ry = vx - wx, vy - wy
            airspeed = math.sqrt(rx * rx + ry * ry + vz * vz)
            payload = b.drone.payload
            b.energy += self.pm.power(payload, airspeed, vz, self._hover_factor(payload)) * dt
            b.dist_h += math.sqrt(vx * vx + vy * vy) * dt
            self.flight_time += dt
            if b.z <= 0.05:
                if b.mode == LAND:
                    b.ground, b.z = True, 0.0
                    b.vx = b.vy = b.vz = 0.0
                    b.hold = None
                    self._release_spot(b)       # taxied off the touchdown spot
                    continue
                b.z = 0.05
            h = heights.get((int(b.x // cm), int(b.y // cm)))
            inside = bool(h) and b.z < (h + 0.5) * lm
            if inside and not b.inside:
                self.building_intrusions += 1
            b.inside = inside

    # --------------------------------------------------------------- monitor
    def _monitor(self, bi: Body, bj: Body, dh2: float, dz: float, now: float, seen: set) -> None:
        cfg = self.cfg
        d3 = math.sqrt(dh2 + dz * dz)
        if d3 < self.min_sep:
            self.min_sep = d3
        key = (bi.i, bj.i)
        if dh2 < cfg.sep_h * cfg.sep_h and abs(dz) < cfg.sep_v:
            seen.add(key)
            rec = self._los_open.get(key)
            if rec is None:
                self._los_open[key] = [now, d3, math.sqrt(dh2), abs(dz)]
                self.los_events += 1
                self.events.append(f"Loss of separation: drones {bi.i} and {bj.i} are {math.sqrt(dh2):.0f} m apart "
                                   f"horizontally and {abs(dz):.0f} m vertically")
            elif d3 < rec[1]:
                rec[1] = d3
            self.los_time += self.dt
            if d3 >= 2 * cfg.drone_radius_m:
                self._touching.discard(key)
            else:
                if key not in self._touching:
                    self._touching.add(key)
                    self.collisions += 1
                    x, y, z = (bi.x + bj.x) / 2, (bi.y + bj.y) / 2, (bi.z + bj.z) / 2
                    cx, cy = int(x // cfg.cell_m), int(y // cfg.cell_m)
                    cz = max(1, int(round(z / cfg.layer_m)))
                    self.coll_log.append([bi.i, bj.i, round(now, 1), round(x), round(y), round(z)])
                    self.sim.collisions.append((int(now // cfg.tick_s) + 1, "contact", (bi.i, bj.i), (cx, cy, cz)))
                    self.events.append(f"!! COLLISION: drones {bi.i} and {bj.i} are {d3:.1f} m apart "
                                       f"at ({cx}, {cy}) on layer {cz}")

    def _close_los(self, key, now: float) -> None:
        rec = self._los_open.pop(key)
        self.los_log.append([key[0], key[1], round(rec[0], 1), round(now, 1), round(rec[1], 1)])

    # --------------------------------------------------------------- control
    def _control(self, b: Body, now: float) -> None:
        cfg = self.cfg
        dt = self.dt
        lm = cfg.layer_m
        ref = b.ref
        if b.ground:
            b.px = b.py = b.pz = 0.0
            b.mode, b.coop = GROUND, True
            if ref is None:
                return
            rx, ry, _, _, _, _, mode, k = ref.at(now)
            if mode != TAKEOFF or now - ref.t[k] > cfg.takeoff_window_s:
                return                   # a late take-off waits for the repaired plan
            site = (int(rx // cfg.cell_m), int(ry // cfg.cell_m))
            spot = self._free_spot(site, b.x, b.y)
            if spot is None or (self.orca and not self._column_clear(self.spots[site][spot], b)):
                self.pad_wait += dt
                return
            self._take_spot(b, (site, spot))
            b.x, b.y = self.spots[site][spot]   # ground handling puts it on a free touchdown spot
            b.ground = False
            b.z = 0.0
            b.vx = b.vy = b.vz = 0.0
            b.wex, b.wey = self.wind.at(b.x, b.y, 5.0, now)
            b.inside = False

        x, y, z = b.x, b.y, b.z
        K, Kv, ab = cfg.track_gain, cfg.track_gain_v, cfg.brake_decel
        if ref is None and b.pad is not None:
            mode = LAND
            rx, ry = b.pad
            rz, rvx, rvy, rvz = lm, 0.0, 0.0, 0.0
            d_stop = math.hypot(rx - x, ry - y)
        elif ref is None:
            mode = HOLD
            if b.hold is None:
                b.hold = (x, y, max(1, round(z / lm)) * lm)
            rx, ry, rz = b.hold
            rvx = rvy = rvz = 0.0
            d_stop = math.hypot(rx - x, ry - y)
        else:
            rx, ry, rz, rvx, rvy, rvz, mode, k = ref.at(now)
            if mode == GROUND:
                mode = LAND              # the plan has us on the pad: finish the landing
            if rvx or rvy or rvz:
                d_stop = math.hypot(ref.x[k + 1] - x, ref.y[k + 1] - y) + ref.stop_d[k + 1]
            else:
                d_stop = math.hypot(rx - x, ry - y)
        if b.spot is not None and mode != LAND and z >= lm - 2.0:
            self._release_spot(b)        # climbed out of the pad column

        if mode == LAND:
            if b.spot is None:
                site = (int(rx // cfg.cell_m), int(ry // cfg.cell_m))
                spot = self._free_spot(site, x, y)
                if spot is not None:
                    self._take_spot(b, (site, spot))
            if b.spot is not None:
                rx, ry = self.spots[b.spot[0]][b.spot[1]]
            else:
                rx, ry = x, y            # every touchdown spot is busy: wait here
            ex, ey = rx - x, ry - y
            dh = math.hypot(ex, ey)
            vxd, vyd = K * ex, K * ey
            cap = math.sqrt(2 * ab * dh) + 0.2
            sp = math.hypot(vxd, vyd)
            if sp > cap:
                vxd, vyd = vxd * cap / sp, vyd * cap / sp
            # in the column over its spot the drone descends vertically and others give way;
            # until then it is ordinary traffic at layer 1
            in_column = b.spot is not None and dh < 8.0 and z < lm - 2.0
            aligned = b.spot is not None and dh < 3.0 and math.hypot(b.vx, b.vy) < 1.0
            if aligned or in_column:
                if not self.orca or self._descent_clear(b):
                    vzd = -min(cfg.max_descent, 0.4 + 0.8 * z)
                else:
                    vzd = 0.0
                    self.pad_wait += dt
            else:
                vzd = Kv * (lm - z)
            b.px, b.py, b.pz = vxd, vyd, max(-cfg.max_descent, min(cfg.max_climb, vzd))
            b.mode, b.coop = LAND, not in_column
            return

        if mode == TAKEOFF:
            vzd = rvz + Kv * (rz - z)
            b.px, b.py, b.pz = 0.0, 0.0, max(0.0, min(cfg.max_climb, vzd))
            b.mode, b.coop = TAKEOFF, z > lm - 2.0
            return

        ex, ey, ez = rx - x, ry - y, rz - z
        vxd, vyd, vzd = rvx + K * ex, rvy + K * ey, rvz + Kv * ez
        if z < 0.6 * lm:
            vxd = vyd = 0.0              # near the ground only vertical motion (climb out first)
        cap = min(cfg.max_speed_h, math.sqrt(2 * ab * d_stop) + 0.3)
        sp = math.hypot(vxd, vyd)
        if sp > cap:
            vxd, vyd = vxd * cap / sp, vyd * cap / sp
        b.px, b.py, b.pz = vxd, vyd, max(-cfg.max_descent, min(cfg.max_climb, vzd))
        # below layer 1 a drone is still climbing out of a pad column: others give way
        b.mode, b.coop = mode, mode != WINCH and z >= lm - 2.0
        if mode in (FLY, WINCH):
            e = math.sqrt(ex * ex + ey * ey + ez * ez)
            self.track_sum += e * dt
            self.track_time += dt
            self.track_hist[min(200, int(e))] += dt

    # ------------------------------------------------------ vertiport (pads)
    def _free_spot(self, site, x: float, y: float) -> int | None:
        """The free touchdown spot of pad ``site`` nearest to (x, y)."""
        best, bd = None, math.inf
        for k, (sx, sy) in enumerate(self.spots.get(site, ())):
            if (site, k) in self.spot_user:
                continue
            dd = (sx - x) ** 2 + (sy - y) ** 2
            if dd < bd:
                best, bd = k, dd
        return best

    def _take_spot(self, b: Body, key) -> None:
        self.spot_user[key] = b.i
        b.spot = key

    def _release_spot(self, b: Body) -> None:
        if b.spot is not None and self.spot_user.get(b.spot) == b.i:
            del self.spot_user[b.spot]
        b.spot = None

    def _column_clear(self, xy, b: Body) -> bool:
        """Vertiport rule: nobody airborne inside the bubble around a spot's column up to layer 1."""
        cfg = self.cfg
        B = self.bucket
        x, y = xy
        gx, gy = int(x // B), int(y // B)
        top = cfg.layer_m + cfg.sep_v
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                for j in self._grid.get((gx + ox, gy + oy), ()):
                    if j is not b and j.z < top and math.hypot(j.x - x, j.y - y) < cfg.sep_h:
                        return False
        return True

    def _descent_clear(self, b: Body) -> bool:
        """Vertiport rule: descend only if nobody is below inside the bubble (plus a 3 m margin)."""
        cfg = self.cfg
        lim = cfg.sep_v + 3.0
        for _, j in b.nb:
            if j.z < b.z and b.z - j.z < lim and math.hypot(j.x - b.x, j.y - b.y) < cfg.sep_h:
                return False
        return True

    # ------------------------------------------------------------------ ORCA
    def _orca(self, b: Body, now: float) -> None:
        cfg = self.cfg
        s = self.s_v
        R = self.R
        tau = cfg.orca_horizon_s
        vmax = cfg.max_speed_h
        nb = b.nb
        if len(nb) > cfg.orca_max_neighbors:
            nb = sorted(nb, key=lambda e: e[0])[:cfg.orca_max_neighbors]
        vi = (b.vx, b.vy, b.vz * s)
        px, py, pz = b.px, b.py, b.pz * s
        planes = list(self.vplanes)
        n_fixed = len(planes) + self._building_planes(b, planes)
        close = ahead = False
        pn = math.sqrt(px * px + py * py + pz * pz)
        for ds2, j in nb:
            dist = math.sqrt(ds2)
            spj = math.sqrt(j.vx * j.vx + j.vy * j.vy + j.vz * j.vz * s * s)
            if dist - R > (vmax + spj) * tau:
                continue                      # no velocity we could pick meets it within the horizon
            rp = (j.x - b.x, j.y - b.y, (j.z - b.z) * s)
            rv = (b.vx - j.vx, b.vy - j.vy, (b.vz - j.vz) * s)
            planes.append(orca_plane(rp, rv, vi, R, tau, self.dt, 0.5 if j.coop else 1.0))
            if dist < 2.5 * R:
                dot = rp[0] * px + rp[1] * py + rp[2] * pz
                ahead = ahead or dot > 0.0
                close = close or dot > 0.9 * dist * pn
        if len(planes) == n_fixed:
            b.cx, b.cy, b.cz = b.px, b.py, b.pz
            self._orca_state(b, False, now)
            return
        blocked = ahead and pn > 5.0 and math.hypot(b.vx, b.vy) < 2.0
        if close or blocked:                  # prefer passing on the right (a roundabout when stuck)
            ang = UNBLOCK if blocked else KEEP_RIGHT
            c, sn = math.cos(ang), math.sin(ang)
            px, py = px * c + py * sn, -px * sn + py * c
        v = solve(planes, (px, py, pz), vmax, n_fixed)
        cx, cy, cz = v[0], v[1], v[2] / s
        cz = max(-cfg.max_descent, min(cfg.max_climb, cz))
        b.cx, b.cy, b.cz = cx, cy, cz
        dv = math.sqrt((cx - b.px) ** 2 + (cy - b.py) ** 2 + (cz - b.pz) ** 2)
        self._orca_state(b, dv > INTERVENTION, now)

    def _building_planes(self, b: Body, planes: list) -> int:
        """Buildings as static obstacles: never close in on a nearby building box faster than
        its distance (minus a margin) allows within ``BUILDING_HORIZON`` seconds."""
        cfg = self.cfg
        cm = cfg.cell_m
        heights = self.world.heights
        cx, cy = int(b.x // cm), int(b.y // cm)
        added = 0
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                h = heights.get((cx + ox, cy + oy))
                if not h or (h + 0.5) * cfg.layer_m + 5.0 < b.z:
                    continue
                x0, y0 = (cx + ox) * cm, (cy + oy) * cm
                qx, qy = min(max(b.x, x0), x0 + cm), min(max(b.y, y0), y0 + cm)
                dx, dy = b.x - qx, b.y - qy
                d = math.hypot(dx, dy)
                if d >= BUILDING_RANGE or d < 1e-6:
                    continue
                nx, ny = dx / d, dy / d
                k = -max(0.0, d - BUILDING_MARGIN) / BUILDING_HORIZON
                planes.append((k * nx, k * ny, 0.0, nx, ny, 0.0))
                added += 1
        return added

    def _orca_state(self, b: Body, on: bool, now: float) -> None:
        if on and not b.orca_on:
            b.orca_on, b.orca_t0 = True, now
            self.orca_events += 1
        elif not on and b.orca_on:
            b.orca_on = False
            if self.record:
                self.orca_log.append([b.i, round(b.orca_t0, 1), round(now, 1)])
        if on:
            b.orca_tick = True

    # ------------------------------------------------------------ tick boundary
    def observe(self, d, t: int) -> None:
        """Report the physical state back to the agent at tick ``t``: cell, battery, statistics."""
        b = self.bodies[d.did]
        if b.dead:
            return
        cfg = self.cfg
        used, b.energy = b.energy, 0.0
        d.pack.charge = max(0.0, d.pack.charge - used)
        st = d.stats
        st["energy_used"] += used
        st["cells_flown"] += b.dist_h / cfg.cell_m
        b.dist_h = 0.0
        step = d.plan.step_at(t) if d.plan is not None else None
        pos = None
        if not b.ground and d.pos[2] == 0 and (b.pad is not None or (step is not None and not step.airborne)):
            pos = d.pos                                 # finishing a descent the agent already counts as landed
        elif step is not None and step.airborne:
            sx, sy, sz = centre(step.cell, cfg)
            dh = math.hypot(b.x - sx, b.y - sy)
            landing = d.plan.steps[-1].tag == "land" and any(
                s.tag in ("land_start", "land") and s.t <= t for s in d.plan.steps)
            if landing and dh <= cfg.track_tol_h:
                pos = step.cell                         # descending onto (or already on) the pad
            elif not b.ground and dh <= cfg.track_tol_h and abs(b.z - sz) <= cfg.track_tol_v:
                pos = step.cell
        if pos is None:
            if b.ground:
                pos = (int(b.x // cfg.cell_m), int(b.y // cfg.cell_m), 0)
            else:
                pos = self._snap(b, d)
        prev = d.pos
        d.pos = pos
        if pos[2] > 0:
            st["airborne_ticks"] += 1
            if pos[2] > 1:
                st["upper_layer_ticks"] += 1
            if prev[2] > 0:
                if pos == prev:
                    st["hover_ticks"] += 1
                elif pos[:2] == prev[:2]:
                    st["climbs" if pos[2] > prev[2] else "descents"] += 1
        if d.pack.charge <= 0.0 and (pos[2] > 0 or not b.ground):
            d._die(t)
            self._release_spot(b)
            b.dead = True
            b.ground = True
            b.z = 0.0
            b.vx = b.vy = b.vz = 0.0

    def _snap(self, b: Body, d) -> tuple[int, int, int]:
        """The open airspace cell the drone is in (or the nearest one)."""
        cfg, w = self.cfg, self.world
        cx, cy = int(b.x // cfg.cell_m), int(b.y // cfg.cell_m)
        cz = min(max(1, int(round(b.z / cfg.layer_m))), w.n_layers)
        if w.is_open((cx, cy, cz)):
            return cx, cy, cz
        best, bd = None, math.inf
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                for z in range(1, w.n_layers + 1):
                    c = (cx + ox, cy + oy, z)
                    if not w.is_open(c):
                        continue
                    x, y, zz = centre(c, cfg)
                    dd = (x - b.x) ** 2 + (y - b.y) ** 2 + (zz - b.z) ** 2
                    if dd < bd:
                        best, bd = c, dd
        return best if best is not None else (d.pos if d.pos[2] > 0 else (d.pos[0], d.pos[1], 1))

    def after_agents(self, t: int) -> None:
        """Deadlock handling: hand drones that make no progress back to the strategic layer."""
        now = t * self.cfg.tick_s
        for b in self.bodies:
            d = b.drone
            if b.dead or not d.alive or not d.airborne or d.plan is None or b.ref is None:
                b.stalls = 0 if (b.dead or not d.airborne) else b.stalls
                continue
            gx, gy, gz = b.ref.goal
            dist = math.sqrt((gx - b.x) ** 2 + (gy - b.y) ** 2 + (gz - b.z) ** 2)
            if d.deviation_streak == 0:
                b.stall_best, b.stall_t, b.stalls = dist, now, 0
            elif dist < b.stall_best - 25.0:
                b.stall_best, b.stall_t = dist, now
            elif now - b.stall_t >= self.cfg.stall_s:
                b.stalls += 1
                self.stall_replans += 1
                d.hold_streak = max(d.hold_streak, b.stalls - 1)
                d._log(t, "has made no progress for a while and asks the planner for a new route")
                d._hold(t)
                b.stall_best, b.stall_t = dist, now

    # ----------------------------------------------------------------- trace
    def _sample(self, now: float) -> None:
        for b in self.bodies:
            q = (int(round(b.x)), int(round(b.y)), int(round(b.z)) if not b.ground else 0)
            if self.samples == 0:
                b.track.extend(q)
            else:
                p = b.q
                b.track.extend((q[0] - p[0], q[1] - p[1], q[2] - p[2]))
            b.q = q
        cx, cy = self.world.width * self.cfg.cell_m / 2, self.world.height * self.cfg.cell_m / 2
        wx, wy = self.wind.at(cx, cy, self.cfg.layer_m, now)
        self.wind_log.extend((int(round(wx * 10)), int(round(wy * 10))))
        self.samples += 1

    def trace(self, end_s: float) -> dict:
        los = self.los_log + [[k[0], k[1], round(r[0], 1), round(end_s, 1), round(r[1], 1)]
                              for k, r in self._los_open.items()]
        orca = self.orca_log + [[b.i, round(b.orca_t0, 1), round(end_s, 1)] for b in self.bodies if b.orca_on]
        cfg = self.cfg
        return {"dt": cfg.trace_dt, "n": self.samples, "cell_m": cfg.cell_m, "layer_m": cfg.layer_m,
                "sep": [cfg.sep_h, cfg.sep_v], "radius": cfg.drone_radius_m,
                "pos": [b.track for b in self.bodies], "wind": self.wind_log,
                "los": sorted(los, key=lambda r: r[2]), "orca": sorted(orca, key=lambda r: r[1]),
                "coll": self.coll_log}

    # --------------------------------------------------------------- metrics
    def metrics(self) -> dict:
        hours = self.flight_time / 3600.0
        p95 = 0.0
        if self.track_time > 0:
            acc, lim = 0.0, 0.95 * self.track_time
            for m, s in enumerate(self.track_hist):
                acc += s
                if acc >= lim:
                    p95 = float(m)
                    break
        return {
            "collisions": self.collisions,
            "vertex_conflicts": 0,
            "edge_conflicts": 0,
            "separation_losses": self.los_events,
            "separation_loss_s": self.los_time,
            "min_separation_m": self.min_sep if self.min_sep < math.inf else self.cfg.sense_radius_m,
            "tracking_error_mean_m": self.track_sum / self.track_time if self.track_time else 0.0,
            "tracking_error_p95_m": p95,
            "orca_interventions": self.orca_events,
            "orca_per_drone_hour": self.orca_events / hours if hours else 0.0,
            "flight_hours": hours,
            "stall_replans": self.stall_replans,
            "pad_wait_s": self.pad_wait,
            "building_intrusions": self.building_intrusions,
        }
