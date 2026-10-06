"""Customers in buildings: parcels winched onto roofs at several heights."""

import contextlib
import dataclasses
import io
import unittest

import run_simulation
from dronefleet import SimConfig, Simulation
from dronefleet.planner import SpaceTimePlanner, Waypoint
from dronefleet.reservation import ReservationTable
from dronefleet.world import GridWorld, generate_world

FAST = SimConfig(record_trace=False)
ROOFS = dataclasses.replace(FAST, customer_buildings=True)


class WorldTests(unittest.TestCase):
    def test_off_by_default(self):
        self.assertFalse(SimConfig().customer_buildings)
        w = generate_world(FAST)
        self.assertEqual(w.customer_heights, {})
        self.assertTrue(all(w.drop_cell(c) == (c[0], c[1], 1) for c in w.customers))

    def test_only_the_customers_change(self):
        for seed in (1, 7):
            plain, roofs = generate_world(dataclasses.replace(FAST, seed=seed)), \
                generate_world(dataclasses.replace(ROOFS, seed=seed))
            self.assertEqual((roofs.customers, roofs.hubs, roofs.stations, roofs.nfzs),
                             (plain.customers, plain.hubs, plain.stations, plain.nfzs))
            self.assertEqual(roofs.blocked - plain.blocked, {c for c, h in roofs.customer_heights.items() if h})
            self.assertTrue(plain.blocked <= roofs.blocked)
            for c, h in roofs.customer_heights.items():
                self.assertIn(h, range(roofs.n_layers))                  # always a layer above the roof
                self.assertEqual(roofs.building_height(c), h)
                self.assertEqual(roofs.drop_cell(c), (c[0], c[1], h + 1))
                self.assertLess(roofs.dist(roofs.hubs[0], roofs.drop_cell(c)), float("inf"))
            self.assertEqual(set(roofs.customer_heights.values()), {0, 1, 2})

    def test_one_layer_means_houses(self):
        w = generate_world(dataclasses.replace(ROOFS, n_layers=1))
        self.assertEqual(set(w.customer_heights.values()), {0})

    def test_drop_is_one_layer_above_the_roof(self):
        w = GridWorld(10, 3, [], hubs=[(0, 0)], stations=[(9, 2)], customers=[(5, 1), (7, 1)], n_layers=3,
                      heights={(5, 1): 2}, customer_heights={(5, 1): 2, (7, 1): 0})
        planner = SpaceTimePlanner(w, ReservationTable())
        for cust, z in (((5, 1), 3), ((7, 1), 1)):
            p = planner.plan(0, (0, 0), 0, False, [Waypoint(cust, "drop", 2), Waypoint((9, 2), "land")], now=0)
            drop = [s.cell for s in p.steps if s.tag in ("drop_start", "drop_done")]
            self.assertEqual(drop, [(cust[0], cust[1], z)] * 2)
            self.assertFalse(any(s.cell[:2] == (5, 1) and s.cell[2] <= 2 for s in p.steps))   # never inside


class SystemTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = []
        for motion in ("grid", "continuous"):
            sim = Simulation(dataclasses.replace(ROOFS, seed=2, motion=motion, record_messages=True,
                                                 cruise_layer=2 if motion == "continuous" else 0))
            cls.runs.append((sim, sim.run()))

    def test_safe_and_every_order_delivered(self):
        for sim, m in self.runs:
            with self.subTest(motion=sim.cfg.motion):
                self.assertEqual((m["collisions"], m["dead_drones"], m["nfz_violations"]), (0, 0, 0))
                self.assertEqual(m["delivered"], m["orders"])
                self.assertEqual(m.get("building_intrusions", 0), 0)

    def test_deliveries_at_several_altitudes(self):
        for sim, m in self.runs:
            with self.subTest(motion=sim.cfg.motion):
                heights = {sim.world.drop_cell(o.dest)[2] for o in sim.dispatcher.orders.values()
                           if o.status == "delivered"}
                self.assertEqual(heights, {1, 2, 3})
                self.assertTrue(0 < m["rooftop_drop_share"] < 1)
                self.assertTrue(30 < m["drop_height_mean_m"] < 90)

    def test_messages_and_replay_name_the_roof(self):
        sim, m = self.runs[0]
        texts = [r[5] for r in sim.msglog.records]
        self.assertTrue(any("(roof at 60 m)" in t for t in texts))
        self.assertTrue(any("(house)" in t for t in texts))
        sim2 = Simulation(dataclasses.replace(ROOFS, seed=2, record_trace=True, max_ticks=20))
        data = sim2.trace.to_dict(sim2.run())
        self.assertEqual(data["world"]["customer_heights"],
                         [sim2.world.customer_heights[tuple(c)] for c in data["world"]["customers"]])
        self.assertNotIn("customer_heights", Simulation(dataclasses.replace(SimConfig(), max_ticks=5)).trace.to_dict({})["world"])


class CliTests(unittest.TestCase):
    def test_on_by_default_in_continuous_mode_only(self):
        for args, on in ((["--motion", "continuous"], True), ([], False),
                         (["--motion", "continuous", "--no-customer-buildings"], False),
                         (["--customer-buildings"], True)):
            with contextlib.redirect_stdout(io.StringIO()) as printed:
                run_simulation.main(args + ["--ticks", "5", "--out", "", "--messages", ""])
            self.assertEqual("Customers:" in printed.getvalue(), on, args)


if __name__ == "__main__":
    unittest.main()
