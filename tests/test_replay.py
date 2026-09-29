"""Replay trace (with altitude) and the 2-D / 3-D HTML exports."""

import contextlib
import io
import json
import re
import tempfile
import unittest
from pathlib import Path

import run_simulation
from dronefleet import SimConfig, Simulation
from dronefleet.replay import render_html

SHORT = SimConfig(max_ticks=80, order_until=60, n_layers=3, layer_rule="heading", seed=2)


class ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation(SHORT)
        cls.metrics = cls.sim.run()
        cls.data = cls.sim.trace.to_dict(cls.metrics)

    def decode(self, e):
        w, h = self.data["world"]["w"], self.data["world"]["h"]
        return e % w, (e // w) % h, e // (w * h)

    def test_trace_records_altitude_and_3d_routes(self):
        world = self.data["world"]
        self.assertEqual((world["layers"], world["layer_rule"]), (3, "heading"))
        self.assertEqual(len(world["heights"]), len(world["blocked"]))
        self.assertTrue(all(1 <= h <= 3 for h in world["heights"]))
        self.assertTrue(all(z["layers"] == [1, 3] for z in world["nfz"]))
        layers = {d[13] for f in self.data["frames"] for d in f["d"]}
        self.assertTrue({0, 1, 2} <= layers, layers)
        for f in self.data["frames"]:
            for d in f["d"]:
                self.assertEqual(bool(d[2]), d[13] > 0)       # airborne <=> above the ground
        cells = [self.decode(e) for plans in self.data["plans"].values() for _, cs in plans for e in cs]
        self.assertTrue(any(c[2] >= 2 for c in cells), "routes use upper layers")
        self.assertTrue(all(0 <= c[2] <= 3 for c in cells))

    def test_2d_export_is_self_contained(self):
        html = render_html(self.data, "2d")
        self.assertNotIn("__REPLAY_DATA__", html)
        self.assertNotIn("__VIEWER_CORE", html)
        self.assertIn("function startReplay", html)
        self.assertIsNone(re.search(r"<script[^>]+src=", html), "the 2-D viewer must work offline")
        payload = re.search(r"const DATA = (\{.*?\});\n", html, re.S).group(1)
        self.assertEqual(json.loads(payload.replace("<\\/", "</"))["world"]["layers"], 3)

    def test_3d_export_pins_one_umd_three_js_version(self):
        html = render_html(self.data, "3d")
        srcs = re.findall(r'<script src="([^"]+)"', html)
        self.assertEqual(len(srcs), 2)
        versions = {re.search(r"cdn\.jsdelivr\.net/npm/three@(\d+\.\d+\.\d+)/", s).group(1) for s in srcs}
        self.assertEqual(len(versions), 1, "three.js and OrbitControls must come from the same release")
        self.assertTrue(any(s.endswith("/build/three.min.js") for s in srcs))
        self.assertTrue(any(s.endswith("/examples/js/controls/OrbitControls.js") for s in srcs))
        self.assertNotIn('type="module"', html)
        self.assertEqual(html.count('integrity="sha384-'), 2)
        self.assertIn("function startReplay", html)
        self.assertNotIn("__VIEWER_CORE", html)

    def test_unknown_view_is_rejected(self):
        with self.assertRaises(ValueError):
            render_html(self.data, "vr")

    def test_cli_view_selection(self):
        self.assertEqual(run_simulation.replay_paths(None, "2d"), [("2d", Path("results/replay.html"))])
        self.assertEqual(run_simulation.replay_paths(None, "3d"), [("3d", Path("results/replay_3d.html"))])
        self.assertEqual(run_simulation.replay_paths("x/run.html", "both"),
                         [("2d", Path("x/run.html")), ("3d", Path("x/run_3d.html"))])
        self.assertEqual(run_simulation.replay_paths("", "3d"), [])
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "r.html"
            with contextlib.redirect_stdout(io.StringIO()):
                run_simulation.main(["--quiet", "--ticks", "40", "--view", "both", "--out", str(out)])
            self.assertTrue(out.exists() and (Path(tmp) / "r_3d.html").exists())


if __name__ == "__main__":
    unittest.main()
