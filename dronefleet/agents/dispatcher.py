"""Dispatcher agent: turns customer orders into drone missions.

Three allocation strategies are implemented so they can be compared:

``cnp`` (default) - Contract Net Protocol (Smith, 1980). Each round the
    dispatcher broadcasts a CFP with a batch of open orders. Every available
    drone evaluates the orders *against its own state* (position, battery,
    whether it is still flying home) and answers with PROPOSE bids or REFUSE.
    Winner determination is a greedy sequential auction: express orders first,
    then by deadline, each going to the cheapest bidder not yet awarded in this
    round. Knowledge stays distributed - the dispatcher never inspects a
    drone's battery.

``nearest`` - centralised baseline: assign each order to the idle drone that
    is closest (Manhattan) to the pickup hub, using broadcast telemetry.

``round_robin`` - centralised baseline: hand orders to idle drones in turn.

In all strategies a drone can decline (REFUSE / FAILURE) an order it cannot
fly safely; the order then returns to the pending pool.
"""

from __future__ import annotations

from ..messages import Performative
from ..orders import Order
from ..world import manhattan
from .base import Agent

AVAILABLE_STATES = ("idle", "returning")


class DispatcherAgent(Agent):
    role = "dispatcher"

    def __init__(self, cfg, bus):
        super().__init__("dispatcher", bus)
        self.cfg = cfg
        self.strategy = cfg.allocation
        self.orders: dict[int, Order] = {}
        self.pending: list[int] = []
        self.round: dict | None = None
        self.round_no = 0
        self.telemetry: dict[str, dict] = {}
        self.rr_next = 0
        self.refused: dict[tuple[int, str], int] = {}   # (oid, drone) -> until_t
        self.awards = 0
        self.failed_awards = 0
        self.next_cfp_t = 0
        self.expired = 0

    # ----------------------------------------------------------------- orders
    def add_order(self, order: Order) -> None:
        self.orders[order.oid] = order
        self.pending.append(order.oid)
        self.next_cfp_t = min(self.next_cfp_t, order.created_t)

    def _requeue(self, oid: int) -> None:
        o = self.orders[oid]
        o.status, o.drone, o.assigned_t = "pending", None, None
        if oid not in self.pending:
            self.pending.append(oid)

    def _sorted_pending(self) -> list[Order]:
        return sorted((self.orders[i] for i in self.pending),
                      key=lambda o: (-o.priority, o.deadline_t, o.oid))

    # ------------------------------------------------------------------- step
    def step(self, t: int) -> list[str]:
        events: list[str] = []
        for msg in self.drain():
            kind = msg.content.get("type")
            p = msg.performative
            if kind == "telemetry":
                self.telemetry[msg.sender] = msg.content
            elif p == Performative.PROPOSE and self.round and msg.conversation == self.round["id"]:
                for b in msg.content["bids"]:
                    self.round["bids"].setdefault(b["oid"], []).append((b["cost"], msg.sender, b["eta"]))
            elif p == Performative.REFUSE and kind == "assign":
                oid = msg.content["oid"]
                self.refused[(oid, msg.sender)] = t + 10
                self._requeue(oid)
                self.failed_awards += 1
            elif p == Performative.FAILURE:
                oid = msg.content["oid"]
                self.failed_awards += 1
                self._requeue(oid)
                self.next_cfp_t = t
                events.append(f"Order #{oid} goes back to the queue: Drone {msg.sender.removeprefix('drone')} "
                              f"can no longer take it ({msg.content.get('reason')})")
            elif kind == "picked_up":
                o = self.orders[msg.content["oid"]]
                o.status, o.picked_t = "picked", t
            elif kind == "delivered":
                o = self.orders[msg.content["oid"]]
                o.status, o.delivered_t = "delivered", t
                late = f", {t - o.deadline_t} ticks LATE" if t > o.deadline_t else ", on time"
                events.append(f"Drone {msg.sender.removeprefix('drone')} delivers order #{o.oid} "
                              f"({t - o.created_t} ticks after it was placed{late})")
            elif kind == "package_lost":
                o = self.orders[msg.content["oid"]]
                o.status = "failed"
                events.append(f"!! Order #{o.oid} is LOST with Drone {msg.sender.removeprefix('drone')}")

        for oid in list(self.pending):
            o = self.orders[oid]
            if t > o.deadline_t + self.cfg.order_expiry:
                o.status = "failed"
                self.pending.remove(oid)
                self.expired += 1
                events.append(f"Order #{oid} is cancelled: no drone could take it in time")
        if self.strategy == "cnp":
            events += self._cnp(t)
        else:
            events += self._centralised(t)
        return events

    # -------------------------------------------------------------------- CNP
    def _cnp(self, t: int) -> list[str]:
        events = []
        if self.round and t > self.round["t"]:
            awarded_before = self.awards
            events += self._award(t)
            self.round = None
            if self.awards == awarded_before:
                self.next_cfp_t = t + self.cfg.cfp_backoff
        if self.round is None and self.pending and t >= self.next_cfp_t:
            batch = self._sorted_pending()[: self.cfg.auction_batch]
            self.round_no += 1
            rid = f"cfp-{self.round_no}"
            self.round = {"id": rid, "t": t, "orders": [o.oid for o in batch], "bids": {}}
            for o in batch:
                o.auctions += 1
            self.send("drones", Performative.CFP, t, rid, type="cfp",
                      orders=[o.to_msg() for o in batch])
        return events

    def _award(self, t: int) -> list[str]:
        events = []
        rnd = self.round
        awarded_drones: set[str] = set()
        bidders = {d for bids in rnd["bids"].values() for _, d, _ in bids}
        for oid in sorted(rnd["orders"], key=lambda i: (-self.orders[i].priority, self.orders[i].deadline_t, i)):
            o = self.orders[oid]
            if o.status != "pending":
                continue
            bids = sorted(b for b in rnd["bids"].get(oid, []) if b[1] not in awarded_drones)
            if not bids:
                continue
            cost, drone, eta = bids[0]
            awarded_drones.add(drone)
            self._assign(o, drone, t)
            self.send(drone, Performative.ACCEPT_PROPOSAL, t, rnd["id"], type="award", order=o.to_msg())
            n = len(rnd["bids"][oid])
            rivals = f"the best of {n} bids" if n > 1 else "the only bid"
            events.append(f"Auction: Drone {drone.removeprefix('drone')} wins order #{oid}"
                          f"{' (EXPRESS)' if o.express else ''} with {rivals}, promising delivery by t={eta}")
        for d in bidders - awarded_drones:
            self.send(d, Performative.REJECT_PROPOSAL, t, rnd["id"], type="award")
        return events

    def _assign(self, o: Order, drone: str, t: int) -> None:
        o.status, o.drone, o.assigned_t = "assigned", int(drone.removeprefix("drone")), t
        self.pending.remove(o.oid)
        self.awards += 1

    # --------------------------------------------------------- centralised
    def _centralised(self, t: int) -> list[str]:
        events = []
        free = {n: tm for n, tm in self.telemetry.items() if tm["state"] in AVAILABLE_STATES
                and tm["soc"] > self.cfg.naive_threshold and not tm.get("task")}
        names = sorted(free, key=lambda n: int(n.removeprefix("drone")))
        for o in self._sorted_pending():
            cands = [n for n in names if self.refused.get((o.oid, n), -1) < t]
            if not cands:
                break
            if self.strategy == "nearest":
                pick = min(cands, key=lambda n: (manhattan(tuple(free[n]["pos"]), o.hub), n))
            else:  # round robin over drone ids
                ids = sorted(int(n.removeprefix("drone")) for n in cands)
                nxt = next((i for i in ids if i >= self.rr_next), ids[0])
                self.rr_next = nxt + 1
                pick = f"drone{nxt}"
            names.remove(pick)
            o.auctions += 1
            self._assign(o, pick, t)
            self.send(pick, Performative.REQUEST, t, f"assign-{o.oid}", type="assign", order=o.to_msg())
            how = "nearest free drone" if self.strategy == "nearest" else "next drone in turn"
            events.append(f"Dispatcher assigns order #{o.oid} to Drone {pick.removeprefix('drone')} ({how})")
        return events
