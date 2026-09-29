"""Altitude layers: 3-D world, planning, conflicts, energy and whole-system runs."""

import dataclasses
import itertools
import unittest

from dronefleet import SimConfig, Simulation
from dronefleet.energy import EnergyModel, step_cost_bound
from dronefleet.planner import PlanStep, SpaceTimePlanner, Waypoint
from dronefleet.reservation import ReservationTable
from dronefleet.traffic import detect_collisions, resolve
from dronefleet.world import GridWorld, NoFlyZone, generate_world

FAST = SimConfig(record_trace=False)


def layered_world(w=10, h=6, n_layers=3, heights=None, nfzs=(), rule="free", hubs=((0, 0),), stations=None):
    stations = stations or [(w - 1, h - 1)]
    return GridWorld(w, h, [], hubs=list(hubs), stations=list(stations), customers=[], nfzs=nfzs,
                     n_layers=n_layers, heights=heights or {}, layer_rule=rule)


def plan(world, start, airborne, waypoints, rt=None, agent=0):
    return SpaceTimePlanner(world, rt or ReservationTable()).plan(agent, start, 0, airborne, waypoints, now=0)


def vertical_steps(p):
    return [(a, b) for a, b in zip(p.steps, p.steps[1:])
            if a.airborne and b.airborne and a.cell[:2] == b.cell[:2] and a.cell[2] != b.cell[2]]


class LayeredWorldTests(unittest.TestCase):
    def test_building_blocks_only_layers_up_to_its_height(self):
        w = layered_world(heights={(2, 0): 1, (3, 0): 3})
        self.assertFalse(w.is_open((2, 0, 1)))
        self.assertTrue(w.is_open((2, 0, 2)))
        self.assertFalse(w.is_open((3, 0, 3)))
        self.assertFalse(w.is_open((4, 0, 4)))          # above the top layer
        self.assertFalse(w.is_open((4, 0, 0)))          # the ground is not airspace

    def test_neighbours_are_four_horizontal_plus_up_and_down(self):
        w = layered_world()
        self.assertEqual(set(w.neighbors((4, 3, 2))),
                         {(5, 3, 2), (3, 3, 2), (4, 4, 2), (4, 2, 2), (4, 3, 3), (4, 3, 1)})
        self.assertNotIn((4, 3, 0), w.neighbors((4, 3, 1)))   # touching down is a landing, not a move
        self.assertNotIn((4, 3, 4), w.neighbors((4, 3, 3)))

    def test_heading_rule_separates_east_west_and_north_south(self):
        w = layered_world(rule="heading")
        odd = {n for n in w.neighbors((4, 3, 1)) if n[2] == 1}
        even = {n for n in w.neighbors((4, 3, 2)) if n[2] == 2}
        self.assertEqual(odd, {(5, 3, 1), (3, 3, 1)})
        self.assertEqual(even, {(4, 4, 2), (4, 2, 2)})
        self.assertEqual(layered_world(n_layers=1, rule="heading").layer_rule, "free")

    def test_heights_do_not_change_the_generated_city(self):
        worlds = {n: generate_world(dataclasses.replace(FAST, seed=4, n_layers=n)) for n in (1, 3, 5)}
        for n, w in worlds.items():
            self.assertEqual(w.blocked, worlds[1].blocked)
            self.assertEqual((w.hubs, w.stations, w.customers), (worlds[1].hubs, worlds[1].stations, worlds[1].customers))
            self.assertEqual([z.rect for z in w.nfzs], [z.rect for z in worlds[1].nfzs])
            self.assertTrue(all(1 <= h <= n for h in w.heights.values()))
        self.assertTrue(any(h < 5 for h in worlds[5].heights.values()), "some buildings can be overflown")

    def test_distance_is_exact_admissible_and_consistent(self):
        cfg = dataclasses.replace(FAST, seed=3, n_layers=3, layer_rule="heading")
        w = generate_world(cfg)
        goal = w.customers[0]
        dm = w.distance_map(goal)
        self.assertEqual(dm[(*goal, 1)], 0)
        for c, d in dm.items():
            for n in w.neighbors(c):
                self.assertLessEqual(abs(d - dm[n]), 1)      # consistent for unit-cost moves
        for site in w.hubs + w.stations:                     # exact: A* with no traffic flies it in h ticks
            p = plan(w, site, False, [Waypoint(goal, "land")])
            self.assertEqual(p.end_t, 1 + dm[(*site, 1)])


class LayeredPlannerTests(unittest.TestCase):
    def test_take_off_climb_and_descend_are_vertical_moves(self):
        w = layered_world()
        p = plan(w, (0, 0, 3), True, [Waypoint((4, 0), "land")])
        self.assertEqual(p.steps[-1].cell, (4, 0, 1))
        self.assertEqual(p.steps[-1].tag, "land")
        self.assertEqual(p.end_t, 4 + 2)                  # 4 moves + 2 descents
        p = plan(w, (0, 0), False, [Waypoint((4, 0), "land")])
        self.assertEqual((p.steps[0].cell, p.steps[1].cell, p.steps[1].tag), ((0, 0, 0), (0, 0, 1), "takeoff"))

    def test_parcels_are_lowered_from_layer_one(self):
        w = layered_world()
        p = plan(w, (0, 0, 3), True, [Waypoint((5, 2), "drop", 2), Waypoint((9, 5), "land")])
        drop = [s for s in p.steps if s.tag in ("drop_start", "drop_done")]
        self.assertTrue(all(s.cell == (5, 2, 1) for s in drop))

    def test_climbs_over_traffic_instead_of_waiting(self):
        w = layered_world(w=6, h=1, n_layers=2, hubs=[(0, 0)], stations=[(5, 0)])
        rt = ReservationTable()
        rt.reserve_path(9, [((2, 0, 1), t, True) for t in range(30)])   # a drone parked over (2, 0)
        p = plan(w, (0, 0), False, [Waypoint((5, 0), "land")], rt)
        self.assertIsNotNone(p)
        self.assertEqual(p.end_t, 1 + 5 + 2)                            # climb + descend, no waiting
        self.assertTrue(vertical_steps(p))
        self.assertFalse(any(s.cell == (2, 0, 1) for s in p.steps))

    def test_overflies_low_buildings_but_not_tall_ones(self):
        wall = {(3, y): 1 for y in range(6)}
        self.assertIsNone(plan(layered_world(n_layers=1, heights=wall), (0, 0), False, [Waypoint((6, 0), "land")]))
        p = plan(layered_world(n_layers=2, heights=wall), (0, 0), False, [Waypoint((6, 0), "land")])
        self.assertIsNotNone(p)
        self.assertEqual({s.cell[2] for s in p.steps if s.cell[:2] == (3, 0)}, {2})
        tall = {(3, y): 2 for y in range(6)}
        self.assertIsNone(plan(layered_world(n_layers=2, heights=tall), (0, 0), False, [Waypoint((6, 0), "land")]))

    def test_prefers_a_short_climb_over_a_long_detour(self):
        # a low wall with a gap far away: going over costs 2 extra ticks, around costs 10
        wall = {(3, y): 1 for y in range(0, 5)}
        w = layered_world(w=8, h=6, n_layers=2, heights=wall)
        p = plan(w, (0, 0), False, [Waypoint((6, 0), "land")])
        self.assertEqual(p.end_t, 1 + 6 + 2)

    def test_never_plans_a_vertical_head_on_swap(self):
        w = layered_world(w=2, h=1, n_layers=3, hubs=[(0, 0)], stations=[(1, 0)])
        rt = ReservationTable()
        other = {0: (0, 0, 1), 1: (0, 0, 2), 2: (0, 0, 3), 3: (0, 0, 3), 4: (0, 0, 3)}   # climbing out
        rt.reserve_path(9, [(c, t, True) for t, c in other.items()])
        self.assertFalse(rt.move_free((0, 0, 2), (0, 0, 1), 0, agent=0))   # the swap itself
        p = plan(w, (0, 0, 2), True, [Waypoint((0, 0), "land")], rt)
        self.assertIsNotNone(p)
        for a, b in zip(p.steps, p.steps[1:]):
            self.assertNotEqual(other.get(b.t), b.cell, "vertex conflict")
            self.assertFalse(other.get(a.t) == b.cell and other.get(b.t) == a.cell, "vertical swap")

    def test_reactive_layer_stops_a_vertical_head_on_pair(self):
        intents = {1: ((4, 4, 1), True, (4, 4, 2), True), 2: ((4, 4, 2), True, (4, 4, 1), True)}
        final, forced = resolve(intents, {1: (0,), 2: (1,)})
        self.assertEqual(forced, {1, 2})
        after = {i: (v[2], v[3]) for i, v in final.items()}
        self.assertEqual(detect_collisions({i: (v[0], v[1]) for i, v in intents.items()}, after), [])
        swapped = {1: ((4, 4, 2), True), 2: ((4, 4, 1), True)}
        kinds = [c[0] for c in detect_collisions({i: (v[0], v[1]) for i, v in intents.items()}, swapped)]
        self.assertEqual(kinds, ["edge"])

    def test_drones_on_different_layers_do_not_conflict(self):
        intents = {1: ((3, 3, 1), True, (4, 3, 1), True), 2: ((5, 3, 2), True, (4, 3, 2), True)}
        _, forced = resolve(intents, {1: (0,), 2: (1,)})
        self.assertEqual(forced, set())

    def test_heading_rule_is_obeyed(self):
        w = layered_world(w=12, h=8, n_layers=3, rule="heading")
        p = plan(w, (0, 0), False, [Waypoint((7, 5), "drop", 2), Waypoint((11, 7), "land")])
        for a, b in zip(p.steps, p.steps[1:]):
            if a.airborne and b.airborne and a.cell[2] == b.cell[2] and a.cell != b.cell:
                east_west = a.cell[0] != b.cell[0]
                self.assertEqual(east_west, a.cell[2] % 2 == 1, f"{a.cell} -> {b.cell}")

    def test_flies_over_a_low_no_fly_zone(self):
        low = NoFlyZone(0, (3, 0, 4, 5), announce_t=0, start_t=0, end_t=100, z0=1, z1=1)
        p = plan(layered_world(n_layers=2, nfzs=[low]), (0, 0), False, [Waypoint((7, 0), "land")])
        self.assertEqual(p.end_t, 1 + 7 + 2)
        for s in p.steps:
            self.assertFalse(s.airborne and low.active(s.t) and low.contains(s.cell))
        full = NoFlyZone(0, (3, 0, 4, 5), announce_t=0, start_t=0, end_t=40)
        p = plan(layered_world(n_layers=2, nfzs=[full]), (0, 0), False, [Waypoint((7, 0), "land")])
        self.assertGreater(p.end_t, 40)                  # every layer is closed: wait it out


class LayeredEnergyTests(unittest.TestCase):
    def test_climb_and_descend_costs(self):
        cfg = SimConfig(n_layers=3)
        e = EnergyModel(cfg)
        steps = [PlanStep(0, (0, 0, 0), False), PlanStep(1, (0, 0, 1), True, "takeoff"),
                 PlanStep(2, (0, 0, 2), True), PlanStep(3, (1, 0, 2), True),
                 PlanStep(4, (1, 0, 1), True, "drop_start"), PlanStep(5, (1, 0, 1), True, "drop_done"),
                 PlanStep(6, (1, 0, 2), True), PlanStep(7, (1, 0, 1), True), PlanStep(8, (1, 0, 0), False)]
        kg = 2.0
        f = 1 + cfg.payload_factor * kg
        expected = ((cfg.takeoff_cost + cfg.climb_cost + cfg.move_cost + cfg.descend_cost + cfg.hover_cost) * f
                    + cfg.climb_cost + cfg.descend_cost + 0.0)
        self.assertAlmostEqual(e.plan_energy(steps, kg), expected)

    def test_estimate_bound(self):
        self.assertEqual(step_cost_bound(SimConfig(n_layers=1, climb_cost=9.0)), SimConfig().move_cost)
        self.assertEqual(step_cost_bound(SimConfig(n_layers=3)), SimConfig().move_cost)   # defaults
        self.assertEqual(step_cost_bound(SimConfig(n_layers=3, climb_cost=3.0, descend_cost=0.5)), 1.75)

    def test_estimates_never_undercount_a_shortest_route(self):
        for climb in (1.2, 3.0):
            cfg = dataclasses.replace(FAST, seed=5, n_layers=5, layer_rule="heading", climb_cost=climb)
            w, e = generate_world(cfg), EnergyModel(cfg)
            sites = w.hubs + w.stations + w.customers[:8]
            for a, b in itertools.permutations(sites, 2):
                p = plan(w, a, False, [Waypoint(b, "land")])
                for kg in (0.0, 2.0):
                    self.assertLessEqual(e.plan_energy(p.steps, kg), e.estimate(w.dist(a, b), kg) + 1e-9)


class LayeredSystemTests(unittest.TestCase):
    """The headline guarantees must hold in stacked airspace too."""

    @classmethod
    def setUpClass(cls):
        cls.runs = []
        for seed, layers, rule in ((1, 3, "free"), (2, 3, "heading"), (3, 5, "heading")):
            sim = Simulation(dataclasses.replace(FAST, seed=seed, n_layers=layers, layer_rule=rule))
            cls.runs.append((sim, sim.run()))

    def test_no_collisions_no_depletion_everything_delivered(self):
        for sim, m in self.runs:
            with self.subTest(seed=sim.cfg.seed, layers=sim.cfg.n_layers, rule=sim.cfg.layer_rule):
                self.assertEqual(m["collisions"], 0)
                self.assertEqual(m["dead_drones"], 0)
                self.assertEqual(m["nfz_violations"], 0)
                self.assertEqual(m["delivered"], m["orders"])
                self.assertTrue(all(d.pack.charge > 0 for d in sim.drones))

    def test_upper_layers_are_used(self):
        for sim, m in self.runs:
            self.assertGreater(m["climbs"], 0)
            self.assertGreater(m["upper_layer_share"], 0)
            self.assertTrue(all(d.pos[2] == 0 for d in sim.drones), "everyone has landed at the end")

    def test_dense_heading_fleet_stays_safe(self):
        sim = Simulation(dataclasses.replace(FAST, seed=4, n_layers=5, layer_rule="heading",
                                             n_drones=32, order_rate=0.47))
        m = sim.run()
        self.assertEqual(m["collisions"], 0)
        self.assertEqual(m["dead_drones"], 0)

    def test_low_no_fly_zones_can_be_overflown(self):
        sim = Simulation(dataclasses.replace(FAST, seed=1, n_layers=3, nfz_ceiling=1))
        self.assertTrue(all(z.layers(3) == (1, 1) for z in sim.world.nfzs))
        m = sim.run()
        self.assertEqual((m["collisions"], m["nfz_violations"], m["dead_drones"]), (0, 0, 0))

    def test_config_validation(self):
        with self.assertRaises(ValueError):
            SimConfig(n_layers=0).validate()
        with self.assertRaises(ValueError):
            SimConfig(layer_rule="diagonal").validate()


if __name__ == "__main__":
    unittest.main()
