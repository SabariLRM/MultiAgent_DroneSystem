import unittest

from dronefleet.planner import SpaceTimePlanner, Waypoint
from dronefleet.reservation import ReservationTable
from dronefleet.world import GridWorld, NoFlyZone


def open_world(w=10, h=6, blocked=(), nfzs=()):
    return GridWorld(w, h, blocked, hubs=[(0, 0)], stations=[(w - 1, h - 1)], customers=[(5, 3)], nfzs=nfzs)


class ReservationTableTests(unittest.TestCase):
    def test_vertex_and_swap_conflicts(self):
        rt = ReservationTable()
        rt.reserve_path(1, [((0, 0), 0, True), ((1, 0), 1, True)])
        self.assertFalse(rt.vertex_free((1, 0), 1, agent=2))
        self.assertTrue(rt.vertex_free((1, 0), 1, agent=1))
        # agent 2 flying (1,0)->(0,0) at t=0 would meet agent 1 head-on
        self.assertFalse(rt.move_free((1, 0), (0, 0), 0, agent=2))
        self.assertTrue(rt.move_free((1, 0), (1, 1), 0, agent=2))

    def test_ground_steps_are_not_reserved(self):
        rt = ReservationTable()
        rt.reserve_path(1, [((0, 0), 0, False), ((0, 0), 1, False), ((0, 0), 2, True)])
        self.assertTrue(rt.vertex_free((0, 0), 1, agent=2))
        self.assertFalse(rt.vertex_free((0, 0), 2, agent=2))

    def test_release_from_time(self):
        rt = ReservationTable()
        rt.reserve_path(1, [((0, 0), t, True) for t in range(5)])
        rt.release(1, from_t=3)
        self.assertFalse(rt.vertex_free((0, 0), 2, agent=2))
        self.assertTrue(rt.vertex_free((0, 0), 3, agent=2))

    def test_hold_reports_bumped_agents(self):
        rt = ReservationTable()
        rt.reserve_path(7, [((2, 2), 5, True)])
        bumped = rt.hold(3, (2, 2), 4, 6)
        self.assertEqual(bumped, {7})
        self.assertEqual(rt.vertex_owner((2, 2), 5), 3)

    def test_disabled_table_never_blocks(self):
        rt = ReservationTable(enabled=False)
        rt.reserve_path(1, [((0, 0), 0, True)])
        self.assertTrue(rt.vertex_free((0, 0), 0, agent=2))
        self.assertEqual(len(rt), 0)

    def test_prune(self):
        rt = ReservationTable()
        rt.reserve_path(1, [((0, 0), t, True) for t in range(10)])
        rt.prune(5)
        self.assertEqual(len(rt), 5)


class PlannerTests(unittest.TestCase):
    def test_shortest_path_with_takeoff_and_landing(self):
        w = open_world()
        p = SpaceTimePlanner(w, ReservationTable())
        plan = p.plan(0, (0, 0), 0, False, [Waypoint((4, 0), "land")], now=0)
        self.assertIsNotNone(plan)
        self.assertTrue(plan.is_contiguous())
        self.assertEqual(plan.steps[1].tag, "takeoff")
        self.assertEqual(plan.steps[-1].tag, "land")
        self.assertEqual(plan.end_t, 1 + 4)          # take-off tick + 4 moves

    def test_drop_dwell_and_chain(self):
        w = open_world()
        p = SpaceTimePlanner(w, ReservationTable())
        plan = p.plan(0, (0, 0), 0, False, [Waypoint((3, 0), "drop", 2), Waypoint((9, 5), "land")], now=0)
        tags = {s.tag: s.t for s in plan.steps if s.tag}
        self.assertEqual(tags["drop_done"] - tags["drop_start"], 2)
        drop_steps = [s for s in plan.steps if tags["drop_start"] <= s.t <= tags["drop_done"]]
        self.assertTrue(all(s.cell == (3, 0, 1) for s in drop_steps))

    def test_avoids_reserved_cells_in_time(self):
        w = open_world(w=6, h=1)   # a 1-wide corridor forces waiting
        rt = ReservationTable()
        # another drone sits over (2,0) during t=0..4
        rt.reserve_path(9, [((2, 0, 1), t, True) for t in range(5)])
        plan = SpaceTimePlanner(w, rt).plan(0, (0, 0), 0, False, [Waypoint((5, 0), "land")], now=0)
        self.assertIsNotNone(plan)
        for s in plan.steps:
            if s.airborne and s.cell == (2, 0, 1):
                self.assertGreaterEqual(s.t, 5)

    def test_never_plans_head_on_swap(self):
        w = open_world(w=6, h=2)
        rt = ReservationTable()
        # drone 9 flies right-to-left along row 0
        rt.reserve_path(9, [((5 - t, 0, 1), t, True) for t in range(6)])
        plan = SpaceTimePlanner(w, rt).plan(0, (0, 0), 0, True, [Waypoint((5, 0), "land")], now=0)
        self.assertIsNotNone(plan)
        other = {t: (5 - t, 0, 1) for t in range(6)}
        for a, b in zip(plan.steps, plan.steps[1:]):
            self.assertNotEqual(other.get(b.t), b.cell, "vertex conflict")
            self.assertFalse(other.get(a.t) == b.cell and other.get(b.t) == a.cell, "swap conflict")

    def test_waits_on_ground_when_pad_airspace_taken(self):
        w = open_world()
        rt = ReservationTable()
        rt.reserve_path(9, [((0, 0, 1), t, True) for t in range(1, 4)])
        plan = SpaceTimePlanner(w, rt).plan(0, (0, 0), 0, False, [Waypoint((3, 0), "land")], now=0)
        takeoff = next(s for s in plan.steps if s.tag == "takeoff")
        self.assertGreaterEqual(takeoff.t, 4)
        self.assertTrue(all(not s.airborne for s in plan.steps if s.t < takeoff.t))

    def test_respects_no_fly_zone_window(self):
        zone = NoFlyZone(0, (4, 2, 6, 4), announce_t=0, start_t=0, end_t=40)
        w = open_world(w=12, h=8, nfzs=[zone])
        p = SpaceTimePlanner(w, ReservationTable())
        plan = p.plan(0, (0, 0), 0, False, [Waypoint((5, 3), "drop", 2), Waypoint((11, 7), "land")], now=0)
        self.assertIsNotNone(plan)
        for s in plan.steps:
            if s.airborne:
                self.assertFalse(zone.active(s.t) and zone.contains(s.cell))
        drop = next(s for s in plan.steps if s.tag == "drop_start")
        self.assertGreaterEqual(drop.t, 40)

    def test_drone_caught_in_an_active_zone_leaves_by_the_shortest_route(self):
        zone = NoFlyZone(0, (3, 2, 7, 6), announce_t=0, start_t=0, end_t=50)
        w = open_world(w=12, h=8, nfzs=[zone])
        plan = SpaceTimePlanner(w, ReservationTable()).plan(0, (5, 4), 0, True, [Waypoint((11, 7), "land")], now=0)
        self.assertIsNotNone(plan)
        inside = [s for s in plan.steps[1:] if s.airborne and zone.active(s.t) and zone.contains(s.cell)]
        self.assertEqual(len(inside), 2)                 # 3 moves to the nearest edge, the last one outside
        self.assertEqual([s.t for s in inside], [1, 2])  # and it never comes back in

    def test_unannounced_zone_is_unknown(self):
        zone = NoFlyZone(0, (2, 0, 2, 5), announce_t=50, start_t=58, end_t=90)
        w = open_world(nfzs=[zone])
        plan = SpaceTimePlanner(w, ReservationTable()).plan(0, (0, 0), 0, False, [Waypoint((4, 0), "land")], now=0)
        self.assertEqual(plan.end_t, 5)   # planner cannot know about it yet

    def test_unreachable_goal(self):
        blocked = [(3, y) for y in range(6)]
        w = GridWorld(10, 6, blocked, hubs=[(0, 0)], stations=[(9, 5)], customers=[])
        plan = SpaceTimePlanner(w, ReservationTable()).plan(0, (0, 0), 0, False, [Waypoint((9, 5), "land")], now=0)
        self.assertIsNone(plan)

    def test_heuristic_is_exact_bfs_distance(self):
        blocked = [(2, y) for y in range(0, 5)]
        w = GridWorld(6, 6, blocked, hubs=[(0, 0)], stations=[(4, 0)], customers=[])
        self.assertEqual(w.dist((0, 0), (4, 0)), 14)


if __name__ == "__main__":
    unittest.main()
