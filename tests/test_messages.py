"""The human-readable message log, the cruise-altitude option and the CLI that writes both."""

import contextlib
import dataclasses
import io
import tempfile
import unittest
from pathlib import Path

import run_simulation
from dronefleet import SimConfig, Simulation
from dronefleet.msglog import KINDS, ROUTINE, agent_name
from dronefleet.planner import SpaceTimePlanner, Waypoint
from dronefleet.reservation import ReservationTable
from dronefleet.world import GridWorld

FAST = SimConfig(record_trace=False)


class MessageLogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation(dataclasses.replace(FAST, seed=3, record_messages=True))
        cls.metrics = cls.sim.run()
        cls.log = cls.sim.msglog

    def test_every_message_is_recorded(self):
        self.assertEqual(len(self.log.records), self.metrics["messages"])
        self.assertEqual(sum(self.log.counts.values()), len(self.log.records))

    def test_every_kind_has_a_plain_english_sentence(self):
        for act, topic in self.log.counts:
            self.assertIn((act, topic), KINDS, f"no label for {act} {topic}")
        for t, sender, receiver, act, topic, text in self.log.records:
            self.assertFalse(text.startswith(f"{topic}:"), f"fell back to raw content: {text}")
            self.assertNotIn("drone", sender.split(" ")[0])          # 'Drone 3', not 'drone3'
            self.assertTrue(text[0].isupper(), text)

    def test_the_main_conversations_are_there(self):
        topics = {topic for _, _, _, _, topic, _ in self.log.records}
        self.assertTrue({"cfp", "bid", "award", "picked_up", "delivered", "telemetry", "swap",
                         "swap_done", "station_status"} <= topics, topics)

    def test_agent_names(self):
        self.assertEqual([agent_name(n) for n in ("dispatcher", "drone11", "station2", "drones", "atc")],
                         ["Dispatcher", "Drone 11", "Station 2", "all drones", "Air traffic control"])

    def test_written_file_lists_messages_in_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            text = self.log.write(Path(tmp) / "m.txt", "Run 3").read_text()
            text_all = self.log.write(Path(tmp) / "all.txt", "Run 3", everything=True).read_text()
        lines = text.splitlines()
        self.assertEqual(lines[0], "Run 3")
        self.assertIn("Messages by kind", text)
        self.assertIn("auction announcements (dispatcher to all drones)", text)
        self.assertIn("CALL FOR BIDS", text)
        self.assertNotIn("Status: ", text)                          # routine reports are only counted ...
        self.assertIn("Status: ", text_all)                         # ... unless everything is asked for
        body = lines[lines.index("Every message") + 4:]
        listed = [r for r in self.log.records if r[4] not in ROUTINE]
        self.assertEqual(len(body), len(listed))
        ticks = [int(line[:5]) for line in body]
        self.assertEqual(ticks, sorted(ticks))

    def test_recording_does_not_change_the_run(self):
        clock = ("planner_ms_per_search", "wall_time_s")
        quiet = Simulation(dataclasses.replace(FAST, seed=3)).run()
        self.assertIsNone(Simulation(FAST).msglog)
        self.assertEqual({k: v for k, v in quiet.items() if k not in clock},
                         {k: v for k, v in self.metrics.items() if k not in clock})

    def test_replay_carries_the_messages(self):
        sim = Simulation(dataclasses.replace(SimConfig(), seed=2, max_ticks=60, order_until=40, record_messages=True))
        data = sim.trace.to_dict(sim.run())
        self.assertTrue(data["messages"])
        self.assertTrue(all(len(m) == 5 for m in data["messages"]))
        self.assertEqual([m[0] for m in data["messages"]], sorted(m[0] for m in data["messages"]))
        self.assertNotIn("messages", Simulation(dataclasses.replace(SimConfig(), max_ticks=5)).trace.to_dict({}))


def open_world(w=12, h=4, n_layers=3):
    return GridWorld(w, h, [], hubs=[(0, 0)], stations=[(w - 1, h - 1)], customers=[], nfzs=(), n_layers=n_layers)


def plan(world, goal, **kw):
    return SpaceTimePlanner(world, ReservationTable(), **kw).plan(0, (0, 0), 0, False, [Waypoint(goal, "land")], now=0)


CRUISE = dict(cruise_layer=2, cruise_penalty=2.5, cruise_high_penalty=0.25, cruise_line_penalty=1.0)


def cruise_layers(p):
    """Layers of the horizontal moves of a plan."""
    return [b.cell[2] for a, b in zip(p.steps, p.steps[1:]) if a.cell[:2] != b.cell[:2]]


class CruiseTests(unittest.TestCase):
    def test_off_by_default(self):
        self.assertEqual(SimConfig().cruise_layer, 0)
        p = plan(open_world(), (10, 0))
        self.assertEqual(set(cruise_layers(p)), {1})
        self.assertEqual(p.end_t, 1 + 10)                          # take off, fly 10 cells

    def test_long_trips_climb_to_the_cruise_layer(self):
        p = plan(open_world(), (10, 0), cruise_layer=3, cruise_penalty=0.6)
        layers = cruise_layers(p)
        self.assertEqual(max(layers), 3)
        self.assertGreaterEqual(layers.count(3), 8)
        self.assertEqual(p.end_t, 1 + 2 + 10 + 2)                  # take off, climb to 90 m, across, back to 30 m
        self.assertEqual(p.steps[-1].cell, (10, 0, 1))

    def test_short_hops_stay_low(self):
        p = plan(open_world(), (2, 0), cruise_layer=3, cruise_penalty=0.6)
        self.assertEqual(set(cruise_layers(p)), {1})

    def test_climbs_over_buildings_and_comes_back_down(self):
        # a row of 30 m buildings across the way at x = 6: 60 m over the streets, 90 m over the roofs
        wall = {(6, y): 1 for y in range(4)}
        w = GridWorld(14, 4, [], hubs=[(0, 0)], stations=[(13, 3)], customers=[], nfzs=(), n_layers=3, heights=wall)
        p = plan(w, (12, 0), **CRUISE)
        layer = {b.cell[0]: b.cell[2] for a, b in zip(p.steps, p.steps[1:]) if a.cell[:2] != b.cell[:2]}
        self.assertEqual(layer[6], 3)
        self.assertEqual([layer[x] for x in (3, 4, 9, 10)], [2, 2, 2, 2])

    def test_flies_over_a_building_in_its_way_rather_than_around(self):
        w = GridWorld(12, 5, [], hubs=[(0, 2)], stations=[(11, 4)], customers=[], nfzs=(), n_layers=3,
                      heights={(5, 2): 1})
        p = SpaceTimePlanner(w, ReservationTable(), **CRUISE).plan(0, (0, 2), 0, False, [Waypoint((10, 2), "land")],
                                                                    now=0)
        cells = [s.cell for s in p.steps]
        self.assertIn((5, 2, 3), cells)                            # straight over the roof, at 90 m
        self.assertEqual({c[1] for c in cells}, {2})               # never left the line

    def test_cruise_layer_is_capped_at_the_top_layer(self):
        planner = SpaceTimePlanner(open_world(n_layers=2), ReservationTable(), cruise_layer=5, cruise_penalty=0.6)
        self.assertEqual(planner.cruise_layer, 2)

    def test_continuous_fleet_cruises_high_and_stays_safe(self):
        cfg = dataclasses.replace(FAST, motion="continuous", seed=5, cruise_layer=3, max_ticks=300, order_until=200)
        m = Simulation(cfg).run()
        self.assertEqual((m["collisions"], m["dead_drones"], m["building_intrusions"]), (0, 0, 0))
        self.assertGreater(m["upper_layer_share"], 0.5)


class CliTests(unittest.TestCase):
    def test_messages_file_is_written_next_to_the_replay(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "run.html"
            with contextlib.redirect_stdout(io.StringIO()) as printed:
                run_simulation.main(["--quiet", "--ticks", "40", "--out", str(out)])
            log = Path(tmp) / "run_messages.txt"
            self.assertTrue(log.exists())
            self.assertIn("Messages between the agents: seed 7", log.read_text())
            self.assertIn(str(log), printed.getvalue())
            self.assertIn('"messages"', out.read_text())

    def test_message_and_cruise_options(self):
        self.assertEqual(run_simulation.messages_path("", []), "")
        self.assertEqual(run_simulation.messages_path(None, []), "results/messages.txt")
        self.assertEqual(run_simulation.messages_path(None, run_simulation.replay_paths("r/x_3d.html", "3d")),
                         "r/x_messages.txt")
        self.assertIsNone(run_simulation.parse_args([]).cruise_layer)
        self.assertEqual({k: getattr(SimConfig(), k) for k in CRUISE},
                         dict(CRUISE, cruise_layer=0))                    # the tests use the defaults
        self.assertEqual(run_simulation.parse_args(["--cruise-layer", "2"]).cruise_layer, 2)
        for args, line in ((["--motion", "continuous"], "Cruise at 60 m"), ([], None),
                           (["--motion", "continuous", "--cruise-layer", "0"], None)):
            with contextlib.redirect_stdout(io.StringIO()) as printed:
                run_simulation.main(args + ["--ticks", "5", "--out", "", "--messages", ""])
            if line:
                self.assertIn(line, printed.getvalue())
            else:
                self.assertNotIn("Cruise", printed.getvalue())


if __name__ == "__main__":
    unittest.main()
