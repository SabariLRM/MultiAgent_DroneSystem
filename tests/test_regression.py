"""Regression guard: the flat (single-altitude) simulator must not drift.

``tests/data/baseline_flat_seeds_1_3.json`` holds the metrics of seeds 1-3
with the default configuration, recorded before altitude layers were added.
With a single flight layer (``n_layers=1``) the layered simulator must
reproduce them exactly (timing metrics excluded).
"""

import dataclasses
import json
import unittest
from pathlib import Path

from dronefleet import SimConfig, Simulation

BASELINE = Path(__file__).with_name("data") / "baseline_flat_seeds_1_3.json"
FLAT = SimConfig(record_trace=False, n_layers=1)


class FlatBaselineRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.baseline = json.loads(BASELINE.read_text())["runs"]

    def test_seeds_1_to_3_reproduce_the_recorded_metrics(self):
        for seed, expected in sorted(self.baseline.items()):
            with self.subTest(seed=seed):
                got = Simulation(dataclasses.replace(FLAT, seed=int(seed))).run()
                self.assertEqual({k: got[k] for k in expected}, expected)


if __name__ == "__main__":
    unittest.main()
