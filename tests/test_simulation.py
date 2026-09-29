"""End-to-end properties of the whole multi-agent system."""

import dataclasses
import unittest

from dronefleet import SimConfig, Simulation
from dronefleet.world import generate_world

FAST = SimConfig(record_trace=False)


def run(**kw):
    sim = Simulation(dataclasses.replace(FAST, **kw))
    return sim, sim.run()


class WorldGenerationTests(unittest.TestCase):
    def test_reproducible_and_connected(self):
        a, b = generate_world(FAST), generate_world(FAST)
        self.assertEqual(a.blocked, b.blocked)
        self.assertEqual(a.customers, b.customers)
        dm = a.distance_map(a.hubs[0])
        for c in a.customers + a.stations + a.hubs:
            self.assertIn((*c, 1), dm, f"{c} unreachable")

    def test_nfz_never_covers_a_pad(self):
        for seed in range(1, 8):
            w = generate_world(dataclasses.replace(FAST, seed=seed))
            for z in w.nfzs:
                self.assertTrue(z.announce_t < z.start_t < z.end_t)
                for p in w.pads:
                    self.assertFalse(z.contains(p))


class CooperativeSystemTests(unittest.TestCase):
    """The headline guarantees of the full system (cooperative + predictive + CNP)."""

    @classmethod
    def setUpClass(cls):
        cls.runs = [run(seed=s) for s in (1, 2, 3)]

    def test_no_collisions(self):
        for sim, m in self.runs:
            self.assertEqual(m["collisions"], 0, f"seed {sim.cfg.seed}: {sim.collisions[:3]}")

    def test_no_drone_runs_out_of_battery(self):
        for sim, m in self.runs:
            self.assertEqual(m["dead_drones"], 0)
            for d in sim.drones:
                self.assertGreater(d.pack.charge, 0)

    def test_every_order_delivered(self):
        for sim, m in self.runs:
            self.assertEqual(m["delivered"], m["orders"], f"seed {sim.cfg.seed}")

    def test_no_airspace_violations(self):
        for _, m in self.runs:
            self.assertEqual(m["nfz_violations"], 0)

    def test_battery_packs_are_conserved(self):
        for sim, _ in self.runs:
            ids = [d.pack.pid for d in sim.drones] + [p.pid for s in sim.stations for p in s.packs]
            ids += [new.pid for s in sim.stations for _, new, _ in s.active.values()]
            self.assertEqual(len(ids), len(set(ids)))
            self.assertEqual(len(ids), sim.cfg.n_drones + sim.cfg.n_stations * sim.cfg.station_spare_packs)

    def test_swaps_actually_happen(self):
        for _, m in self.runs:
            self.assertGreater(m["swaps"], 0)

    def test_deterministic(self):
        _, again = run(seed=1)
        first = dict(self.runs[0][1])
        for m in (first, again):
            m.pop("wall_time_s")
            m.pop("planner_ms_per_search")
        self.assertEqual(first, again)


class BaselineContrastTests(unittest.TestCase):
    def test_uncoordinated_flight_collides(self):
        _, m = run(seed=2, coordination="none", n_drones=20, order_rate=0.3)
        self.assertGreater(m["collisions"], 0)

    def test_reactive_layer_alone_prevents_collisions(self):
        _, m = run(seed=2, coordination="reactive", n_drones=20, order_rate=0.3)
        self.assertEqual(m["collisions"], 0)
        self.assertGreater(m["forced_holds"], 0)

    def test_strong_gusts_do_not_cause_collisions(self):
        _, m = run(seed=4, gust_prob=0.25)
        self.assertEqual(m["collisions"], 0)
        self.assertGreater(m["deviations"], 0)

    def test_all_allocation_strategies_complete(self):
        for alloc in ("nearest", "round_robin"):
            _, m = run(seed=3, allocation=alloc)
            self.assertEqual(m["collisions"], 0)
            self.assertGreater(m["delivery_rate"], 0.95)


if __name__ == "__main__":
    unittest.main()
