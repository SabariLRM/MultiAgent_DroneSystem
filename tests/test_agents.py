import unittest

from dronefleet.agents.dispatcher import DispatcherAgent
from dronefleet.agents.drone import DroneAgent, DroneState
from dronefleet.agents.station import SwapStationAgent
from dronefleet.config import SimConfig
from dronefleet.energy import BatteryPack, EnergyModel
from dronefleet.messages import Message, MessageBus, Performative
from dronefleet.orders import Order
from dronefleet.planner import PlanStep, SpaceTimePlanner
from dronefleet.reservation import ReservationTable
from dronefleet.traffic import detect_collisions, resolve
from dronefleet.world import GridWorld


class Sink:
    """A stand-in agent that just collects messages."""

    def __init__(self, bus, name, role):
        from collections import deque
        self.inbox = deque()
        bus.register(name, role, self)


class TrafficTests(unittest.TestCase):
    def test_same_cell_highest_rank_wins(self):
        intents = {1: ((0, 0), True, (1, 0), True), 2: ((2, 0), True, (1, 0), True)}
        final, forced = resolve(intents, {1: (0,), 2: (1,)})
        self.assertEqual(forced, {1})
        self.assertEqual(final[2][2], (1, 0))
        self.assertEqual(final[1][2], (0, 0))

    def test_hovering_drone_keeps_its_cell(self):
        intents = {1: ((1, 0), True, (1, 0), True), 2: ((0, 0), True, (1, 0), True)}
        _, forced = resolve(intents, {1: (0,), 2: (9,)})
        self.assertEqual(forced, {2})

    def test_head_on_swap_is_prevented(self):
        # the loser holds, so the winner's target stays occupied: both stop
        intents = {1: ((0, 0), True, (1, 0), True), 2: ((1, 0), True, (0, 0), True)}
        final, forced = resolve(intents, {1: (0,), 2: (1,)})
        self.assertEqual(forced, {1, 2})
        self.assertEqual(detect_collisions({i: (v[0], v[1]) for i, v in intents.items()},
                                           {i: (v[2], v[3]) for i, v in final.items()}), [])

    def test_blocking_cascades(self):
        # 3 is forced to hold, so 2 (following into 3's cell) must hold too
        intents = {1: ((5, 0), True, (4, 0), True), 3: ((3, 0), True, (4, 0), True),
                   2: ((2, 0), True, (3, 0), True)}
        final, forced = resolve(intents, {1: (5,), 3: (1,), 2: (0,)})
        self.assertEqual(forced, {2, 3})
        cells = [v[2] for v in final.values() if v[3]]
        self.assertEqual(len(cells), len(set(cells)))

    def test_rotation_is_allowed(self):
        intents = {1: ((0, 0), True, (1, 0), True), 2: ((1, 0), True, (1, 1), True),
                   3: ((1, 1), True, (0, 1), True), 4: ((0, 1), True, (0, 0), True)}
        _, forced = resolve(intents, {i: (i,) for i in intents})
        self.assertEqual(forced, set())

    def test_takeoff_yields_to_traffic_over_pad(self):
        intents = {1: ((0, 0), False, (0, 0), True), 2: ((1, 0), True, (0, 0), True)}
        final, forced = resolve(intents, {1: (9,), 2: (0,)})
        self.assertEqual(forced, {1})
        self.assertFalse(final[1][3])   # still on the ground

    def test_landing_frees_the_cell(self):
        intents = {1: ((0, 0), True, (0, 0), False), 2: ((1, 0), True, (0, 0), True)}
        _, forced = resolve(intents, {1: (0,), 2: (0,)})
        self.assertEqual(forced, set())

    def test_detects_vertex_and_edge_collisions(self):
        before = {1: ((0, 0), True), 2: ((1, 0), True), 3: ((5, 5), True), 4: ((5, 7), True)}
        after = {1: ((1, 0), True), 2: ((0, 0), True), 3: ((5, 6), True), 4: ((5, 6), True)}
        kinds = sorted(c[0] for c in detect_collisions(before, after))
        self.assertEqual(kinds, ["edge", "vertex"])


class EnergyTests(unittest.TestCase):
    def test_plan_energy_releases_payload_at_drop(self):
        cfg = SimConfig()
        e = EnergyModel(cfg)
        steps = [PlanStep(0, (0, 0), False), PlanStep(1, (0, 0), True, "takeoff"),
                 PlanStep(2, (1, 0), True, "drop_start"), PlanStep(3, (1, 0), True, "drop_done"),
                 PlanStep(4, (2, 0), True, "land")]
        kg = 2.0
        f = 1 + cfg.payload_factor * kg
        expected = cfg.takeoff_cost * f + cfg.move_cost * f + cfg.hover_cost * f + cfg.move_cost
        self.assertAlmostEqual(e.plan_energy(steps, kg), expected)

    def test_ground_is_free(self):
        e = EnergyModel(SimConfig())
        self.assertEqual(e.transition(False, (0, 0), False, (0, 0), 3.0), 0.0)


class StationTests(unittest.TestCase):
    def setUp(self):
        self.cfg = SimConfig(station_bays=1, swap_ticks=2)
        self.bus = MessageBus()
        self.drones = [Sink(self.bus, f"drone{i}", "drones") for i in range(2)]
        packs = [BatteryPack(100 + i, 150, 150) for i in range(2)]
        self.st = SwapStationAgent(0, (0, 0), self.cfg, self.bus, packs)

    def _request(self, i, charge, t):
        pack = BatteryPack(i, 150, charge)
        self.bus.send(Message(f"drone{i}", "station0", Performative.REQUEST, {"type": "swap", "pack": pack}, t))
        return pack

    def test_fifo_swaps_and_pack_conservation(self):
        self._request(0, 10, 0)
        self._request(1, 20, 0)
        done = {}
        for t in range(0, 10):
            self.st.step(t)
            for i, d in enumerate(self.drones):
                for m in list(d.inbox):
                    if m.content.get("type") == "swap_done":
                        done[i] = t
                d.inbox.clear()
        self.assertLess(done[0], done[1])                   # first come, first served
        self.assertEqual(done[1] - done[0], self.cfg.swap_ticks)  # one bay
        self.assertEqual(len(self.st.packs), 2)              # packs are exchanged, not created
        self.assertEqual(self.st.swaps_done, 2)

    def test_waits_for_a_charged_pack(self):
        for p in self.st.packs:
            p.charge = 0
        self._request(0, 5, 0)
        for t in range(0, 20):
            self.st.step(t)
        self.assertEqual(self.st.swaps_done, 0)              # 20 ticks * 2.0 < 95 % of 150
        for t in range(20, 90):
            self.st.step(t)
        self.assertEqual(self.st.swaps_done, 1)


class DispatcherTests(unittest.TestCase):
    def setUp(self):
        self.cfg = SimConfig(allocation="cnp")
        self.bus = MessageBus()
        self.sinks = [Sink(self.bus, f"drone{i}", "drones") for i in range(3)]
        self.disp = DispatcherAgent(self.cfg, self.bus)

    def _order(self, oid, express=False, t=0):
        return Order(oid, (0, 0), (5, 5), 1.0, express, t, t + (70 if express else 140))

    def _bid(self, drone, oid, cost, rid):
        self.bus.send(Message(f"drone{drone}", "dispatcher", Performative.PROPOSE,
                              {"type": "bid", "bids": [{"oid": oid, "cost": cost, "eta": 10}]}, 0, rid))

    def test_cfp_then_lowest_bid_wins(self):
        self.disp.add_order(self._order(1))
        self.disp.step(0)
        cfp = [m for m in self.sinks[0].inbox if m.performative == Performative.CFP]
        self.assertEqual(len(cfp), 1)
        rid = cfp[0].conversation
        self._bid(0, 1, 30.0, rid)
        self._bid(1, 1, 12.0, rid)
        self._bid(2, 1, 20.0, rid)
        self.disp.step(1)
        o = self.disp.orders[1]
        self.assertEqual((o.status, o.drone), ("assigned", 1))
        self.assertTrue(any(m.performative == Performative.ACCEPT_PROPOSAL for m in self.sinks[1].inbox))
        self.assertTrue(any(m.performative == Performative.REJECT_PROPOSAL for m in self.sinks[0].inbox))

    def test_express_first_and_one_order_per_drone(self):
        self.disp.add_order(self._order(1))
        self.disp.add_order(self._order(2, express=True))
        self.disp.step(0)
        rid = self.disp.round["id"]
        # drone0 is cheapest for both; express order 2 must get it
        for oid in (1, 2):
            self._bid(0, oid, 5.0, rid)
            self._bid(1, oid, 9.0, rid)
        self.disp.step(1)
        self.assertEqual(self.disp.orders[2].drone, 0)
        self.assertEqual(self.disp.orders[1].drone, 1)

    def test_failure_requeues(self):
        self.disp.add_order(self._order(1))
        self.disp.step(0)
        self._bid(2, 1, 5.0, self.disp.round["id"])
        self.disp.step(1)
        self.bus.send(Message("drone2", "dispatcher", Performative.FAILURE, {"type": "award", "oid": 1}, 1))
        self.disp.step(2)
        self.assertIn(1, self.disp.pending)
        self.assertEqual(self.disp.orders[1].status, "pending")


class DroneBiddingTests(unittest.TestCase):
    def _drone(self, charge):
        cfg = SimConfig()
        w = GridWorld(30, 10, [], hubs=[(1, 1)], stations=[(3, 1), (28, 8)], customers=[(25, 5)])
        bus = MessageBus()
        Sink(bus, "dispatcher", "dispatcher")
        rt = ReservationTable()
        d = DroneAgent(0, cfg, bus, w, SpaceTimePlanner(w, rt), rt, EnergyModel(cfg), (1, 1),
                       BatteryPack(1, cfg.battery_capacity, charge))
        return d

    def test_direct_bid_when_battery_suffices(self):
        d = self._drone(150)
        ev = d._evaluate({"oid": 1, "hub": (1, 1), "dest": (25, 5), "weight": 1.0, "deadline_t": 200}, 0)
        self.assertIsNotNone(ev)
        self.assertIsNone(ev["via"])

    def test_swap_first_bid_when_battery_low(self):
        d = self._drone(30)
        ev = d._evaluate({"oid": 1, "hub": (1, 1), "dest": (25, 5), "weight": 1.0, "deadline_t": 200}, 0)
        self.assertIsNotNone(ev)
        self.assertEqual(ev["via"], 0)          # the station next to the hub

    def test_no_bid_when_nothing_is_reachable(self):
        d = self._drone(2)
        ev = d._evaluate({"oid": 1, "hub": (1, 1), "dest": (25, 5), "weight": 1.0, "deadline_t": 200}, 0)
        self.assertIsNone(ev)

    def test_right_of_way_prefers_loaded_express(self):
        a, b = self._drone(100), self._drone(100)
        b.did = 5
        b.task, b.carrying = {"oid": 1, "express": True, "weight": 1.0}, True
        self.assertGreater(b.rank(), a.rank())
        self.assertEqual(a.state, DroneState.IDLE)


if __name__ == "__main__":
    unittest.main()
