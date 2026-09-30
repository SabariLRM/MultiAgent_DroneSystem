"""Continuous 3-D flight: controller, ORCA, energy, wind, smoothing and whole-system runs."""

import dataclasses
import math
import random
import unittest

from dronefleet import SimConfig, Simulation
from dronefleet.flight import FLY, HOLD, FlightLayer, Reference, build_reference, segment_clear
from dronefleet.orca import orca_plane, solve
from dronefleet.planner import Plan, PlanStep, SpaceTimePlanner, Waypoint
from dronefleet.power import ContinuousEnergyModel, PowerModel
from dronefleet.reservation import ReservationTable
from dronefleet.wind import WIND_PRESETS, WindField
from dronefleet.world import GridWorld

CALM = SimConfig(motion="continuous", wind_mean=0.0, wind_gust=0.0, record_trace=False)
MODERATE = SimConfig(motion="continuous", wind_mean=WIND_PRESETS["moderate"][0],
                     wind_gust=WIND_PRESETS["moderate"][1], record_trace=False)
EPS = 1e-6


class StubDrone:
    """Just what the flight layer reads from a drone agent."""

    def __init__(self, did, pos):
        self.did, self.pos, self.plan, self.payload = did, pos, None, 0.0


class StubSim:
    def __init__(self, cfg, world, starts):
        self.cfg, self.world, self.wind = cfg, world, WindField(cfg)
        self.drones = [StubDrone(i, p) for i, p in enumerate(starts)]
        self.collisions = []


def open_world(w=20, h=20, heights=None):
    heights = heights or {}
    return GridWorld(w, h, list(heights), hubs=[(1, 1)], stations=[(w - 2, h - 2)], customers=[],
                     n_layers=3, heights=heights)


def free_flight(starts, goals, cfg=CALM, speed=10.0, t_end=200.0, size=20):
    """Fly straight-line 4-D references (all starting at t = 0) through the flight layer.

    Returns the layer, the largest final distance to a goal and the largest
    horizontal speed, vertical speed and acceleration seen.
    """
    fl = FlightLayer(StubSim(cfg, open_world(size, size), [(0, 0, 0)] * len(starts)))
    for b, s, g in zip(fl.bodies, starts, goals):
        dur = math.dist(s, g) / speed
        b.ground, (b.x, b.y, b.z) = False, s
        b.ref = Reference([0.0, dur], [s[0], g[0]], [s[1], g[1]], [s[2], g[2]], [FLY], HOLD, g)
        b.plan = b.drone.plan = object()
    vmax = vzmax = amax = 0.0
    for k in range(int(t_end / cfg.physics_dt)):
        prev = [(b.vx, b.vy, b.vz) for b in fl.bodies]
        fl._substep(k * cfg.physics_dt)
        for b, p in zip(fl.bodies, prev):
            vmax = max(vmax, math.hypot(b.vx, b.vy))
            vzmax = max(vzmax, b.vz, -b.vz * cfg.max_climb / cfg.max_descent)
            amax = max(amax, math.dist((b.vx, b.vy, b.vz), p) / cfg.physics_dt)
    err = max(math.dist((b.x, b.y, b.z), g) for b, g in zip(fl.bodies, goals))
    return fl, err, vmax, vzmax, amax


class ControllerTests(unittest.TestCase):
    """A planned route is flown within tolerance and within the speed/acceleration limits."""

    @classmethod
    def setUpClass(cls):
        cfg = CALM
        # a wall of 3-layer buildings forces a detour with two corners
        heights = {(7, y): 3 for y in range(0, 6)}
        world = GridWorld(16, 9, list(heights), hubs=[(1, 1)], stations=[(13, 7)], customers=[(10, 2)],
                          n_layers=3, heights=heights)
        planner = SpaceTimePlanner(world, ReservationTable(), cfg.max_expansions, land_dwell=cfg.land_ticks)
        cls.plan = planner.plan(0, (1, 1), 0, False, [Waypoint((10, 2), "drop", 2), Waypoint((13, 7), "land")], now=0)
        cls.cfg, cls.world = cfg, world
        cls.fl = fl = FlightLayer(StubSim(cfg, world, [(1, 1, 0)]))
        fl.bodies[0].drone.plan = cls.plan
        b = fl.bodies[0]
        cls.samples, cls.at_tick = [], {}
        prev = (0.0, 0.0, 0.0)
        for t in range(cls.plan.end_t + 4):
            for k in range(fl.n_sub):
                fl._substep(t * cfg.tick_s + k * cfg.physics_dt)
                cls.samples.append((b.vx, b.vy, b.vz, math.dist((b.vx, b.vy, b.vz), prev) / cfg.physics_dt))
                prev = (b.vx, b.vy, b.vz)
            cls.at_tick[t + 1] = (b.x, b.y, b.z, b.ground)

    def test_waypoints_are_reached_on_time(self):
        cfg = self.cfg
        checked = 0
        for prev, step in zip(self.plan.steps, self.plan.steps[1:]):
            if not (prev.airborne and step.airborne):
                continue                    # take-off: the climb is checked below
            x, y, z, _ = self.at_tick[step.t]
            cx, cy, cz = (step.cell[0] + .5) * cfg.cell_m, (step.cell[1] + .5) * cfg.cell_m, step.cell[2] * cfg.layer_m
            with self.subTest(t=step.t, cell=step.cell):
                self.assertLessEqual(math.hypot(x - cx, y - cy), cfg.track_tol_h)
                self.assertLessEqual(abs(z - cz), cfg.track_tol_v)
            checked += 1
        self.assertGreater(checked, 10)

    def test_route_turns_are_smoothed(self):
        ref = build_reference(self.plan, self.cfg, self.world)
        self.assertLess(len(ref.t), len(self.plan.steps), "smoothing should drop staircase corners")

    def test_lands_at_the_station(self):
        x, y, z, grounded = self.at_tick[self.plan.end_t + 4]
        self.assertTrue(grounded)
        self.assertEqual((int(x // 100), int(y // 100)), (13, 7))
        self.assertEqual(self.fl.building_intrusions, 0)

    def test_speed_and_acceleration_limits(self):
        cfg = self.cfg
        for vx, vy, vz, a in self.samples:
            self.assertLessEqual(math.hypot(vx, vy), cfg.max_speed_h + EPS)
            self.assertLessEqual(vz, cfg.max_climb + EPS)
            self.assertGreaterEqual(vz, -cfg.max_descent - EPS)
            self.assertLessEqual(a, cfg.max_accel + EPS)
        self.assertGreater(max(math.hypot(s[0], s[1]) for s in self.samples), 9.0)   # it did cruise


class OrcaTests(unittest.TestCase):
    """Conflicting straight-line routes, no strategic deconfliction: ORCA alone keeps them apart."""

    C, Z = 1000.0, 30.0

    def check(self, fl, err, vmax, vzmax, amax):
        cfg = CALM
        self.assertEqual(fl.collisions, 0)
        self.assertEqual(fl.los_events, 0, "separation bubble violated")
        self.assertGreaterEqual(fl.min_sep, cfg.sep_h)
        self.assertLess(err, 5.0, "every drone reaches its goal")
        self.assertLessEqual(vmax, cfg.max_speed_h + EPS)
        self.assertLessEqual(vzmax, cfg.max_climb + EPS)
        self.assertLessEqual(amax, cfg.max_accel + EPS)
        self.assertGreater(fl.orca_events, 0)

    def test_head_on_pair(self):
        C, Z = self.C, self.Z
        self.check(*free_flight([(C - 400, C, Z), (C + 400, C, Z)], [(C + 400, C, Z), (C - 400, C, Z)]))

    def test_perpendicular_crossing(self):
        C, Z = self.C, self.Z
        self.check(*free_flight([(C - 400, C, Z), (C, C - 400, Z)], [(C + 400, C, Z), (C, C + 400, Z)]))

    def test_circle_swap(self):
        C, Z = self.C, self.Z
        for n in (4, 8, 12):
            with self.subTest(drones=n):
                starts = [(C + 400 * math.cos(2 * math.pi * i / n), C + 400 * math.sin(2 * math.pi * i / n), Z)
                          for i in range(n)]
                goals = [(2 * C - x, 2 * C - y, Z) for x, y, _ in starts]
                self.check(*free_flight(starts, goals, t_end=300.0))

    def test_layers_do_not_interact(self):
        # 30 m apart vertically is outside the bubble: no avoidance at all
        C = self.C
        fl, err, *_ = free_flight([(C - 400, C, 30.0), (C + 400, C, 60.0)], [(C + 400, C, 30.0), (C - 400, C, 60.0)])
        self.assertEqual(fl.orca_events, 0)
        self.assertLess(err, 1.0)

    def test_orca_plane_is_reciprocal(self):
        # two agents head-on at 10 m/s, 100 m apart: both planes forbid keeping the current velocity
        p = orca_plane((100.0, 0.0, 0.0), (20.0, 0.0, 0.0), (10.0, 0.0, 0.0), 50.0, 8.0, 0.5)
        v = solve([p], (10.0, 0.0, 0.0), 15.0)
        self.assertGreaterEqual((v[0] - p[0]) * p[3] + (v[1] - p[1]) * p[4] + (v[2] - p[2]) * p[5], -1e-9)
        self.assertGreater(math.dist(v, (10.0, 0.0, 0.0)), 0.5)


class EnergyModelTests(unittest.TestCase):
    def setUp(self):
        self.pm = PowerModel(CALM)
        self.e = ContinuousEnergyModel(CALM, WindField(CALM))

    def test_one_cell_at_cruise_costs_one_unit(self):
        self.assertAlmostEqual(self.pm.power(0.0, 10.0, 0.0) * 10.0, 1.0, places=9)
        self.assertAlmostEqual(self.e.move(), 1.0, places=9)
        for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            self.assertAlmostEqual(self.e.transition(True, (5, 5, 1), True, (5 + d[0], 5 + d[1], 1), 0.0), 1.0)

    def test_flown_cruise_matches_the_calibration(self):
        # steady flight along a 1 km leg: energy per 100 m within 2 % of 1.0
        fl = FlightLayer(StubSim(CALM, open_world(), [(0, 0, 0)]))
        b = fl.bodies[0]
        b.ground, b.x, b.y, b.z, b.vx = False, 400.0, 500.0, 30.0, 10.0
        b.ref = Reference([0.0, 100.0], [400.0, 1400.0], [500.0, 500.0], [30.0, 30.0], [FLY], HOLD, (1400, 500, 30))
        b.plan = b.drone.plan = object()
        for k in range(160):                      # 80 s, before the braking into the stop
            fl._substep(k * CALM.physics_dt)
        self.assertAlmostEqual(b.energy / (b.dist_h / 100.0), 1.0, delta=0.02)

    def test_hover_climb_descent(self):
        self.assertAlmostEqual(self.e.hover(), 0.7, places=9)
        self.assertAlmostEqual(self.e.climb(), 1.2, delta=0.02)
        self.assertAlmostEqual(self.e.descend(), 0.55, delta=0.02)
        self.assertGreater(self.e.takeoff(), self.e.climb())       # it also accelerates
        self.assertLess(self.e.descend(), self.e.hover() * CALM.layer_m / CALM.max_descent / CALM.tick_s)

    def test_payload_scales_hover_power_by_mass_to_the_1_5(self):
        for kg in (0.5, 1.0, 2.5):
            with self.subTest(kg=kg):
                ratio = ((CALM.base_mass_kg + kg) / CALM.base_mass_kg) ** 1.5
                self.assertAlmostEqual(self.e.hover(kg) / self.e.hover(0.0), ratio, places=9)
                self.assertGreater(self.e.move(kg), self.e.move(0.0))
                # lifting a heavier drone costs more on top of the heavier hover
                self.assertGreater(self.e.climb(kg) - self.e.hover(kg) * CALM.layer_m / CALM.max_climb / CALM.tick_s,
                                   self.e.climb(0.0) - self.e.hover(0.0) * CALM.layer_m / CALM.max_climb / CALM.tick_s)

    def test_drag_grows_with_airspeed_cubed(self):
        base = self.pm.power(0.0, 0.0, 0.0)
        self.assertAlmostEqual((self.pm.power(0.0, 20.0, 0.0) - base) / (self.pm.power(0.0, 10.0, 0.0) - base), 8.0)

    def test_wind_makes_headwind_cells_dearer(self):
        cfg = dataclasses.replace(MODERATE, wind_dir_deg=0.0)       # the air moves toward +x
        e = ContinuousEnergyModel(cfg, WindField(cfg))
        tail = e.transition(True, (5, 5, 1), True, (6, 5, 1), 0.0)
        head = e.transition(True, (5, 5, 1), True, (4, 5, 1), 0.0)
        cross = e.transition(True, (5, 5, 1), True, (5, 6, 1), 0.0)
        self.assertLess(tail, cross)
        self.assertLess(cross, head)
        strong = dataclasses.replace(cfg, wind_mean=10.0, wind_gust=3.0)
        es = ContinuousEnergyModel(strong, WindField(strong))
        drag, seconds = es._leg(1, -1.0, 0.0)
        self.assertGreater(seconds, strong.tick_s, "into a strong wind the airspeed limit slows the drone")


class WindTests(unittest.TestCase):
    def test_deterministic_for_a_seed(self):
        a, b = WindField(MODERATE), WindField(MODERATE)
        c = WindField(dataclasses.replace(MODERATE, seed=MODERATE.seed + 1))
        pts = [(123.0, 456.0, 30.0, 10.0), (2000.0, 900.0, 60.0, 333.0)]
        self.assertEqual([a.at(*p) for p in pts], [b.at(*p) for p in pts])
        self.assertNotEqual([a.at(*p) for p in pts], [c.at(*p) for p in pts])

    def test_mean_gusts_and_shear(self):
        w = WindField(MODERATE)
        rng = random.Random(0)
        mx, my = w.mean_at(30.0)
        self.assertAlmostEqual(math.hypot(mx, my), MODERATE.wind_mean)
        sq = []
        for _ in range(3000):
            x, y, t = rng.uniform(0, 3200), rng.uniform(0, 2400), rng.uniform(0, 9000)
            wx, wy = w.at(x, y, 30.0, t)
            sq.append((wx - mx) ** 2 + (wy - my) ** 2)
        self.assertAlmostEqual(math.sqrt(sum(sq) / len(sq)), MODERATE.wind_gust, delta=0.2 * MODERATE.wind_gust)
        self.assertGreater(math.hypot(*w.mean_at(90.0)), math.hypot(*w.mean_at(30.0)))

    def test_gusts_are_smooth(self):
        w = WindField(MODERATE)
        for t in (0.0, 100.0, 1000.0):
            a, b = w.at(800.0, 800.0, 30.0, t), w.at(800.0, 800.0, 30.0, t + 0.5)
            self.assertLess(math.dist(a, b), 0.3)          # no white noise: correlated in time ...
            c = w.at(805.0, 800.0, 30.0, t)
            self.assertLess(math.dist(a, c), 0.3)          # ... and in space

    def test_calm_is_calm(self):
        self.assertEqual(WindField(CALM).at(100.0, 100.0, 30.0, 50.0), (0.0, 0.0))

    def test_drones_reach_their_goals_in_moderate_wind(self):
        C, Z = 1000.0, 30.0
        starts = [(C - 500, C - 100, Z), (C + 500, C + 100, Z), (C - 100, C + 500, Z)]
        goals = [(C + 500, C - 100, Z), (C - 500, C + 100, Z), (C - 100, C - 500, Z)]
        fl, err, *_ = free_flight(starts, goals, cfg=MODERATE, t_end=200.0)
        self.assertLess(err, 10.0)
        self.assertEqual(fl.collisions, 0)
        self.assertEqual(fl.los_events, 0)
        self.assertGreater(fl.track_sum / fl.track_time, 0.05, "gusts push the drones off their track")


class SmoothingTests(unittest.TestCase):
    def test_shortcuts_stay_in_the_reserved_tube_and_clear_of_buildings(self):
        cfg = CALM
        heights = {(4, 3): 1, (5, 5): 3, (8, 2): 2}
        world = open_world(12, 10, heights)
        cells = [(1, 1), (2, 1), (2, 2), (3, 2), (3, 3), (3, 4), (4, 4), (5, 4), (6, 4), (6, 5), (6, 6), (7, 6)]
        steps = [PlanStep(t, (x, y, 1), True) for t, (x, y) in enumerate(cells)]
        plan = Plan(steps)
        ref = build_reference(plan, cfg, world)
        self.assertLess(len(ref.t), len(steps))
        for k in range(len(ref.t) - 1):
            self.assertTrue(segment_clear(world, cfg, ref.x[k], ref.y[k], ref.z[k], ref.x[k + 1], ref.y[k + 1], ref.z[k + 1]))
        for s in range(2 * (len(steps) - 1) + 1):          # every half tick
            t = s / 2
            a, b = steps[int(t)], steps[min(len(steps) - 1, int(t) + 1)]
            f = t - int(t)
            raw = [((a.cell[i] + (b.cell[i] - a.cell[i]) * f) + .5) * cfg.cell_m for i in (0, 1)]
            rx, ry, *_ = ref.at(t * cfg.tick_s)
            self.assertLessEqual(math.hypot(rx - raw[0], ry - raw[1]), cfg.smooth_tol * cfg.cell_m + 1e-6)

    def test_no_corner_cutting_past_a_building(self):
        cfg = CALM
        world = open_world(6, 6, {(1, 2): 3})
        # east then north around the building's corner at (1, 2)
        steps = [PlanStep(0, (1, 1, 1), True), PlanStep(1, (2, 1, 1), True), PlanStep(2, (2, 2, 1), True)]
        self.assertFalse(segment_clear(world, cfg, 150, 150, 30, 250, 250, 30))
        ref = build_reference(Plan(steps), cfg, world)
        for k in range(len(ref.t) - 1):
            self.assertTrue(segment_clear(world, cfg, ref.x[k], ref.y[k], ref.z[k], ref.x[k + 1], ref.y[k + 1], ref.z[k + 1]))


class PlannerHookTests(unittest.TestCase):
    def test_landing_dwell_only_when_asked(self):
        world = open_world(8, 8)
        grid = SpaceTimePlanner(world, ReservationTable()).plan(0, (1, 1), 0, False, [Waypoint((6, 6), "land")], now=0)
        self.assertNotIn("land_start", [s.tag for s in grid.steps])
        cont = SpaceTimePlanner(world, ReservationTable(), land_dwell=2).plan(
            0, (1, 1), 0, False, [Waypoint((6, 6), "land")], now=0)
        tags = [s.tag for s in cont.steps]
        self.assertEqual(tags[-3:], ["land_start", None, "land"])
        self.assertEqual(cont.end_t, grid.end_t + 2)
        self.assertEqual({s.cell for s in cont.steps[-3:]}, {(6, 6, 1)})


class ContinuousSystemTests(unittest.TestCase):
    """Continuous mode, default configuration (moderate wind, 12 drones, ORCA + reservations)."""

    @classmethod
    def setUpClass(cls):
        cls.runs = []
        for s in (1, 2, 3):
            sim = Simulation(dataclasses.replace(SimConfig(record_trace=False, motion="continuous"), seed=s))
            cls.runs.append((sim, sim.run()))

    def test_no_collisions_and_no_drone_lost(self):
        for sim, m in self.runs:
            with self.subTest(seed=sim.cfg.seed):
                self.assertEqual(m["collisions"], 0)
                self.assertEqual(m["dead_drones"], 0)
                self.assertEqual(m["building_intrusions"], 0)
                self.assertTrue(all(d.pack.charge > 0 for d in sim.drones))

    def test_every_order_delivered(self):
        for sim, m in self.runs:
            self.assertEqual(m["delivered"], m["orders"], f"seed {sim.cfg.seed}")

    def test_safety_metrics_are_reported(self):
        for _, m in self.runs:
            self.assertGreater(m["flight_hours"], 1.0)
            self.assertGreater(m["min_separation_m"], 2 * CALM.drone_radius_m)
            self.assertGreaterEqual(m["separation_losses"], 0)
            self.assertLess(m["tracking_error_mean_m"], 15.0)
            self.assertGreater(m["orca_per_drone_hour"], 0.0)

    def test_deterministic(self):
        sim = Simulation(dataclasses.replace(SimConfig(record_trace=False, motion="continuous"), seed=1))
        again = sim.run()
        first = dict(self.runs[0][1])
        for m in (first, again):
            m.pop("wall_time_s")
            m.pop("planner_ms_per_search")
        self.assertEqual(first, again)

    def test_battery_packs_are_conserved(self):
        for sim, _ in self.runs:
            ids = [d.pack.pid for d in sim.drones] + [p.pid for s in sim.stations for p in s.packs]
            ids += [new.pid for s in sim.stations for _, new, _ in s.active.values()]
            self.assertEqual(len(ids), len(set(ids)))
            self.assertEqual(len(ids), sim.cfg.n_drones + sim.cfg.n_stations * sim.cfg.station_spare_packs)

    def test_grid_mode_has_no_flight_layer(self):
        sim = Simulation(SimConfig(record_trace=False, max_ticks=30, order_until=20))
        m = sim.run()
        self.assertIsNone(sim.flight)
        self.assertNotIn("separation_losses", m)


class ContinuousReplayTests(unittest.TestCase):
    """The trace of a continuous run and its 2-D / 3-D replays."""

    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation(SimConfig(motion="continuous", seed=2, max_ticks=120, order_until=80))
        cls.metrics = cls.sim.run()
        cls.data = cls.sim.trace.to_dict(cls.metrics)

    def positions(self, k):
        """Decode drone k's delta-encoded track into absolute (x, y, z) metres."""
        arr, out, acc = self.data["track"]["pos"][k], [], [0, 0, 0]
        for j in range(0, len(arr), 3):
            acc = [acc[0] + arr[j], acc[1] + arr[j + 1], acc[2] + arr[j + 2]]
            out.append(tuple(acc))
        return out

    def test_track_samples_every_two_seconds(self):
        tr = self.data["track"]
        self.assertEqual(tr["dt"], 2.0)
        self.assertEqual(tr["n"], int(self.sim.t * self.sim.cfg.tick_s / tr["dt"]) + 1)
        self.assertEqual(len(tr["pos"]), self.sim.cfg.n_drones)
        self.assertTrue(all(len(p) == 3 * tr["n"] and all(isinstance(v, int) for v in p) for p in tr["pos"]))
        self.assertEqual(len(tr["wind"]), 2 * tr["n"])
        self.assertEqual(self.data["meta"]["motion"], "continuous")

    def test_track_matches_the_physics(self):
        for k, b in enumerate(self.sim.flight.bodies):
            last = self.positions(k)[-1]
            self.assertLessEqual(math.dist(last, (b.x, b.y, 0.0 if b.ground else b.z)), 1.0)
            steps = [math.dist(p, q) for p, q in zip(self.positions(k), self.positions(k)[1:])]
            self.assertLessEqual(max(steps), 2.0 * (CALM.max_speed_h + 10.0))    # smooth: no jumps
        flying = [p for k in range(self.sim.cfg.n_drones) for p in self.positions(k) if p[2] > 0]
        self.assertTrue(any(25 <= p[2] <= 35 for p in flying), "cruising at layer 1 (30 m)")

    def test_replays_are_small_and_self_contained(self):
        import re
        from dronefleet.replay import render_html
        full = Simulation(SimConfig(motion="continuous", seed=1))
        data = full.trace.to_dict(full.run())
        html2, html3 = render_html(data, "2d"), render_html(data, "3d")
        for html in (html2, html3):
            self.assertLess(len(html.encode()), 5_000_000)
            self.assertIn("function trackAt", html)
        self.assertIsNone(re.search(r"<script[^>]+src=", html2), "the 2-D viewer must work offline")
        self.assertIn('id="optBubbles"', html3)
        self.assertIn('id="optArrows"', html3)

    def test_grid_replays_have_no_track(self):
        grid = Simulation(SimConfig(max_ticks=40, order_until=30, seed=2))
        data = grid.trace.to_dict(grid.run())
        self.assertNotIn("track", data)
        self.assertNotIn("motion", data["meta"])


if __name__ == "__main__":
    unittest.main()
