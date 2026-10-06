"""The drone agent.

Architecture: a hybrid (layered) agent.

* **Deliberative layer** - a finite-state mission controller with explicit
  beliefs (own battery and position, the latest station broadcasts, known
  no-fly zones) that decides *what* to do next: bid on orders, fly to a hub,
  deliver, go and swap a battery. It plans *how* with space-time A* against
  the shared reservation table, and checks every plan against its energy
  budget before committing (predictive battery policy). Its position is an
  airspace cell ``(x, y, z)``: ``z = 0`` on the ground, ``z >= 1`` airborne.
* **Reactive layer** - executed by :mod:`dronefleet.traffic` every tick: if the
  next cell is contested the lower-priority drone holds position. When that
  (or a wind gust) makes the drone fall behind its plan, the deliberative layer
  repairs or re-plans on the next tick.

State machine::

    IDLE --award--> TO_PICKUP --land--> LOADING --> TO_CUSTOMER --drop--> RETURNING --land--> IDLE
      |                                                  |    \\                    |
      +--low battery--> TO_STATION --land--> QUEUED --> SWAPPING --> IDLE      (emergency divert)
"""

from __future__ import annotations

import math
import random
from enum import Enum

from ..energy import BatteryPack, EnergyModel
from ..messages import Performative
from ..planner import Plan, PlanStep, SpaceTimePlanner, Waypoint
from ..reservation import ReservationTable
from ..world import Cell, GridWorld, Site, ground
from .base import Agent
from .station import READY_SOC


class DroneState(str, Enum):
    IDLE = "idle"
    TO_PICKUP = "to_pickup"
    LOADING = "loading"
    TO_CUSTOMER = "to_customer"
    RETURNING = "returning"
    TO_STATION = "to_station"
    QUEUED = "queued"
    SWAPPING = "swapping"
    DEAD = "dead"


AIR_STATES = {DroneState.TO_PICKUP, DroneState.TO_CUSTOMER, DroneState.RETURNING, DroneState.TO_STATION}
SWAP_STATES = (DroneState.TO_STATION, DroneState.QUEUED, DroneState.SWAPPING)

# why a drone is on a battery trip (shown in the replay)
TRIP_NONE, TRIP_LOW, TRIP_BEFORE_JOB, TRIP_EMERGENCY, TRIP_AFTER_DELIVERY = range(5)


class DroneAgent(Agent):
    role = "drones"

    def __init__(self, did: int, cfg, bus, world: GridWorld, planner: SpaceTimePlanner,
                 reservations: ReservationTable, energy: EnergyModel, start: Site, pack: BatteryPack,
                 sensor=None):
        super().__init__(f"drone{did}", bus)
        self.did = did
        self.cfg = cfg
        self.world = world
        self.planner = planner
        self.res = reservations
        self.energy = energy
        self.sensor = sensor               # callable(drone) -> set of (cell, t) seen occupied
        self.pos: Cell = ground(start)     # parked on its start pad
        self.pack = pack
        self.state = DroneState.IDLE
        self.task: dict | None = None
        self.carrying = False
        self.plan: Plan | None = None
        self.post_pad: Cell | None = None
        self.station_sid: int | None = None
        self.load_until = 0
        self.needs_replan: str | None = None
        self.hold_streak = 0
        self.escalated = False
        self.bid_pending_until = -1
        self.deviation_streak = 0
        self.idle_since = 0
        self.swap_first = False
        self.swap_first_sid = 0
        self.rng = random.Random(cfg.seed * 1000 + did)
        self.trip_reason = TRIP_NONE
        self.peers: list["DroneAgent"] = []     # fleet roster (for right-of-way comparisons)

        # beliefs about the rest of the world, built only from messages
        self.station_beliefs: dict[int, dict] = {}

        # statistics
        self.stats = dict(replans=0, repairs=0, holds=0, deviations=0, yields=0, escalations=0,
                          energy_used=0.0, cells_flown=0, hover_ticks=0, deliveries=0, swaps=0,
                          emergencies=0, bids=0, busy_ticks=0, airborne_ticks=0, plan_failures=0,
                          climbs=0, descents=0, upper_layer_ticks=0, over_building_ticks=0,
                          over_building_top_ticks=0)
        self.plan_log: list[tuple[int, list]] = []       # (t, cells) for the replay viewer
        self.event_log: list[str] = []

    # ================================================================ helpers
    @property
    def airborne(self) -> bool:
        return self.pos[2] > 0

    @property
    def site(self) -> Site:
        """The ground location below the drone."""
        return self.pos[0], self.pos[1]

    @property
    def battery(self) -> float:
        return self.pack.charge

    @property
    def soc(self) -> float:
        return self.pack.charge / self.cfg.battery_capacity

    @property
    def payload(self) -> float:
        return self.task["weight"] if (self.carrying and self.task) else 0.0

    @property
    def alive(self) -> bool:
        return self.state != DroneState.DEAD

    def rank(self) -> tuple:
        """Right-of-way ordering used by the reactive layer and escalation."""
        express = bool(self.carrying and self.task and self.task["express"])
        return (self.escalated, express, self.carrying, -self.did)

    def _log(self, t: int, text: str) -> None:
        self.event_log.append(f"Drone {self.did} {text}")

    def _station_energy(self, frm, takeoff: bool = True) -> float:
        """Energy to reach the nearest swap station from ``frm`` (0 if already there).

        ``frm`` is a site, or a cell (on a station pad or at layer 1 above it
        counts as there). A drone must never land somewhere it cannot fly out
        of to recharge, so every plan is checked against:
        battery >= plan + this + reserve.
        """
        if (frm[0], frm[1]) in self.world.stations and (len(frm) == 2 or frm[2] <= 1):
            return 0.0
        d = min(self.world.dist(frm, s) for s in self.world.stations)
        return self.energy.estimate(d, takeoff=takeoff, slack=self.cfg.detour_factor)

    def _nearest_hub(self, frm) -> Site:
        return min(self.world.hubs, key=lambda h: (self.world.dist(frm, h), h))

    def _in_drop(self, t: int) -> bool:
        """True while hovering over the customer lowering the parcel."""
        if not self.plan or not self.carrying:
            return False
        s = self.plan.step_at(t)
        if s is None or s.cell != self.world.drop_cell(self.task["dest"]) or not self.airborne:
            return False
        started = any(p.tag == "drop_start" and p.t <= t for p in self.plan.steps)
        return started

    # ======================================================= message handling
    def deliberate(self, t: int) -> None:
        if not self.alive:
            self.drain()
            return
        for msg in self.drain():
            self._handle(msg, t)
        self._act(t)
        self.send("dispatcher", Performative.INFORM, t, type="telemetry", state=self.state.value,
                  pos=self.site, alt=self.pos[2], soc=round(self.soc, 3),
                  task=self.task["oid"] if self.task else None)

    def process_urgent(self, t: int) -> None:
        """Handle same-tick yield requests (called by the simulation in extra rounds)."""
        if not self.alive:
            return
        for msg in self.drain():
            self._handle(msg, t)
        if self.needs_replan and self.state in AIR_STATES:
            self._replan(t)

    def _handle(self, msg, t: int) -> None:
        kind = msg.content.get("type")
        p = msg.performative
        if p == Performative.CFP:
            self._bid(msg, t)
        elif p == Performative.ACCEPT_PROPOSAL or (p == Performative.REQUEST and kind == "assign"):
            self._accept(msg, t)
        elif kind == "station_status":
            self.station_beliefs[msg.content["station"]] = msg.content
        elif kind == "swap_started":
            self.state = DroneState.SWAPPING
        elif kind == "swap_done":
            self.pack = msg.content["pack"]
            self.swap_first = False
            self.stats["swaps"] += 1
            self.state = DroneState.IDLE
            self.idle_since = t
            self.station_sid = None
            self._log(t, f"leaves Station {msg.content['station']} with a fresh battery ({self.soc:.0%})")
            self.trip_reason = TRIP_NONE
        elif kind == "yield":
            if self.state in AIR_STATES and not self._in_drop(t):
                self.needs_replan = self.needs_replan or "yield"
                self.stats["yields"] += 1
        elif kind == "nfz":
            if self.plan and self.state in AIR_STATES:
                x0, y0, x1, y1 = msg.content["rect"]
                z0, z1 = msg.content["layers"]
                s, e = msg.content["start_t"], msg.content["end_t"]
                hit = any(s <= st.t < e and x0 <= st.cell[0] <= x1 and y0 <= st.cell[1] <= y1
                          and z0 <= st.cell[2] <= z1 for st in self.plan.remaining(t))
                if hit:
                    self.needs_replan = "nfz"
                    self._log(t, "re-routes around the newly announced no-fly zone")

    # ============================================================== auctions
    def can_bid(self) -> bool:
        if not self.alive or self.task is not None:
            return False
        if self.state in (DroneState.IDLE, DroneState.RETURNING):
            return True
        # on the way to / at a station: bid for work that starts after the swap
        return self.state in SWAP_STATES and self.cfg.battery_policy == "predictive"

    def _availability(self, t: int) -> tuple[tuple, int, float, bool]:
        """(where, when, battery, airborne) this drone could start a new mission from."""
        if self.state in (DroneState.IDLE, DroneState.RETURNING):
            return self.pos, t, self.battery, self.airborne
        cell = self.world.stations[self.station_sid]
        belief = self.station_beliefs.get(self.station_sid, {})
        wait = belief.get("est_wait_now", 0)
        if self.state == DroneState.SWAPPING:
            ready = t + self.cfg.swap_ticks
        elif self.state == DroneState.QUEUED:
            ready = t + wait + self.cfg.swap_ticks
        else:
            arrive = self.plan.end_t if self.plan else t + int(self.world.dist(self.pos, cell))
            ready = arrive + max(0, wait - (arrive - t)) + self.cfg.swap_ticks
        return cell, ready, READY_SOC * self.cfg.battery_capacity, False

    def _leg_estimate(self, start, t0: int, airborne: bool, order: dict) -> tuple[float, int] | None:
        """(energy, eta_delivery) for start -> hub -> customer -> nearest station."""
        w, e = self.world, self.energy
        slack = self.cfg.detour_factor
        hub, dest, kg = tuple(order["hub"]), w.drop_cell(order["dest"]), order["weight"]
        if kg > self.cfg.max_payload_kg:
            return None
        d1 = 0 if ((start[0], start[1]) == hub and not airborne) else w.dist(start, hub)
        d2 = w.dist(hub, dest)
        d3 = min(w.dist(dest, s) for s in w.stations)   # must always be able to recharge
        if math.inf in (d1, d2, d3):
            return None
        energy = 0.0
        if d1:
            energy += e.estimate(d1, 0.0, takeoff=not airborne, slack=slack)
        energy += e.estimate(d2, kg, takeoff=True, hover_ticks=self.cfg.drop_ticks, slack=slack)
        energy += e.estimate(d3, 0.0, takeoff=False, slack=slack)
        to_hub = (d1 + (0 if airborne else 1)) if d1 else 0
        eta = t0 + to_hub + self.cfg.loading_ticks + 1 + d2 + self.cfg.drop_ticks
        return energy, int(eta)

    def _evaluate(self, order: dict, t: int) -> dict | None:
        """Best feasible way for *this* drone to do ``order`` (private knowledge).

        Returns ``{"energy", "eta", "via"}`` where ``via`` is a station id if
        the drone must swap its battery first, or ``None``.
        """
        start, t0, battery, air = self._availability(t)
        est = self._leg_estimate(start, t0, air, order)
        if est is None:
            return None
        energy, eta = est
        if self.cfg.battery_policy == "naive":
            ok = self.soc >= self.cfg.naive_threshold
            return {"energy": energy, "eta": eta, "via": None} if ok else None
        if energy + self.energy.reserve <= battery:
            return {"energy": energy, "eta": eta, "via": None}
        if self.state not in (DroneState.IDLE, DroneState.RETURNING):
            return None
        # not enough charge: offer to do it via a battery swap first
        best = None
        for sid, cell in enumerate(self.world.stations):
            here = self.site == cell and not self.airborne
            d = 0 if here else self.world.dist(self.pos, cell)
            reach = 0.0 if here else self.energy.estimate(d, takeoff=not self.airborne, slack=self.cfg.detour_factor)
            if reach + self.energy.critical > self.battery:
                continue
            wait = self.station_beliefs.get(sid, {}).get("est_wait_now", 0)
            arrive = t + int(d) + (0 if (self.airborne or here) else 1)
            ready = arrive + max(0, wait - (arrive - t)) + self.cfg.swap_ticks
            est2 = self._leg_estimate(cell, ready, False, order)
            if est2 is None or est2[0] + self.energy.reserve > READY_SOC * self.cfg.battery_capacity:
                continue
            if best is None or est2[1] < best["eta"]:
                best = {"energy": reach + est2[0], "eta": est2[1], "via": sid}
        return best

    def _bid(self, msg, t: int) -> None:
        if not self.can_bid():
            return
        bids = []
        for order in msg.content["orders"]:
            ev = self._evaluate(order, t)
            if ev is None:
                continue
            lateness = max(0, ev["eta"] - order["deadline_t"])
            cost = (ev["eta"] - t) + 3.0 * lateness + 5.0 * ev["energy"] / self.cfg.battery_capacity
            bids.append({"oid": order["oid"], "cost": round(cost, 3), "eta": ev["eta"], "via": ev["via"]})
        if bids:
            self.stats["bids"] += len(bids)
            self.bid_pending_until = t + 1
            self.send("dispatcher", Performative.PROPOSE, t, msg.conversation, type="bid", bids=bids)
        else:
            self.send("dispatcher", Performative.REFUSE, t, msg.conversation, type="bid")

    def _accept(self, msg, t: int) -> None:
        order = msg.content["order"]
        order = {**order, "hub": tuple(order["hub"]), "dest": tuple(order["dest"])}
        kind = msg.content.get("type")
        ev = self._evaluate(order, t) if self.can_bid() else None
        if ev is None:
            if kind == "assign":
                self.send("dispatcher", Performative.REFUSE, t, msg.conversation, type="assign", oid=order["oid"])
            else:
                self.send("dispatcher", Performative.FAILURE, t, msg.conversation, type="award",
                          oid=order["oid"], reason="no longer available")
            return
        if kind == "assign":
            self.send("dispatcher", Performative.AGREE, t, msg.conversation, type="assign", oid=order["oid"])
        self.task = order
        self.bid_pending_until = -1
        hub = self.world.hubs.index(order["hub"])
        if ev["via"] is not None:
            self._log(t, f"takes order #{order['oid']}: will swap its battery at Station {ev['via']} first, "
                         f"then collect the parcel at Hub {hub}")
        elif self.state in SWAP_STATES:
            self._log(t, f"takes order #{order['oid']}: will collect it at Hub {hub} once its battery swap is done")
        if self.state in SWAP_STATES:
            return                                   # mission starts when the swap is done
        if ev["via"] is not None:
            self.swap_first, self.swap_first_sid = True, ev["via"]
            self._go_swap(t, self.world.stations[ev["via"]], TRIP_BEFORE_JOB)
        elif self.state == DroneState.RETURNING:
            self.state = DroneState.TO_PICKUP
            self.needs_replan = "new task"

    def _drop_task(self, t: int, reason: str) -> None:
        if self.task is None:
            return
        self.send("dispatcher", Performative.FAILURE, t, type="task", oid=self.task["oid"], reason=reason)
        self._log(t, f"hands order #{self.task['oid']} back to the dispatcher ({reason})")
        self.task = None

    # ============================================================ deliberation
    def _act(self, t: int) -> None:
        st = self.state
        if st != DroneState.IDLE:
            self.stats["busy_ticks"] += 1
        if st == DroneState.IDLE:
            self._act_idle(t)
        elif st == DroneState.LOADING:
            if t >= self.load_until:
                if not self.carrying:
                    self.carrying = True
                    self.send("dispatcher", Performative.INFORM, t, type="picked_up", oid=self.task["oid"])
                    self._log(t, f"picks up order #{self.task['oid']} at Hub {self.world.hubs.index(self.site)}")
                self._start_delivery_leg(t)
        elif st in AIR_STATES:
            if self._battery_emergency(t):
                return
            if self.plan is None or self.needs_replan:
                self._replan(t)

    def _act_idle(self, t: int) -> None:
        if self.task is not None:
            if self.carrying:                       # resuming after an emergency swap
                self._start_delivery_leg(t)
            elif self.swap_first:                   # won the job on a "swap first" bid
                self._go_swap(t, self.world.stations[self.swap_first_sid], TRIP_BEFORE_JOB)
            elif self.site == self.task["hub"]:
                self.state = DroneState.LOADING
                self.load_until = t + self.cfg.loading_ticks
            else:
                self.state = DroneState.TO_PICKUP
                if not self._replan(t):
                    self.state = DroneState.IDLE    # stay on the ground, try again next tick
            return
        if t <= self.bid_pending_until:
            return                                  # wait for the auction result first
        policy = self.cfg.battery_policy
        low = self.soc < (self.cfg.swap_threshold if policy == "predictive" else self.cfg.naive_threshold)
        if low and self.site not in self.world.stations:
            self._go_swap(t)
        elif low and self.site in self.world.stations and self.soc < 0.9:
            # already sitting on a station: just queue here
            sid = self.world.stations.index(self.site)
            self.station_sid = sid
            self.state = DroneState.QUEUED
            self.trip_reason = TRIP_LOW
            self.send(f"station{sid}", Performative.REQUEST, t, type="swap", pack=self.pack)
            self._log(t, f"is low on battery ({self.soc:.0%}) and queues for a swap here at Station {sid}")
        elif self.site not in self.world.hubs and t - self.idle_since >= self.cfg.reposition_after:
            # parked at a station with a good battery: reposition to where parcels are
            self._reposition(t)

    def _reposition(self, t: int) -> None:
        hub = self._nearest_hub(self.pos)
        need = self.energy.estimate(self.world.dist(self.pos, hub), slack=self.cfg.detour_factor) \
            + self._station_energy(hub)
        if need + self.energy.reserve > self.battery:
            return
        self.post_pad, self.station_sid = hub, None
        self.state = DroneState.RETURNING
        if self._replan(t):
            self._log(t, f"flies back to Hub {self.world.hubs.index(hub)} to wait for the next order")
        else:
            self.state = DroneState.IDLE

    def _start_delivery_leg(self, t: int) -> None:
        """Plan customer drop + landing somewhere sensible, check energy, commit."""
        dest = self.task["dest"]
        e = self.energy
        after = self.battery - e.estimate(self.world.dist(self.pos, self.world.drop_cell(dest)), self.payload,
                                          takeoff=not self.airborne, hover_ticks=self.cfg.drop_ticks,
                                          slack=self.cfg.detour_factor)
        options = self._post_delivery_pads(self.world.drop_cell(dest), after)
        energy_short = False
        for pad, is_station in options:
            wps = [Waypoint(dest, "drop", self.cfg.drop_ticks), Waypoint(pad, "land")]
            plan = self._make_plan(t, wps)
            if plan is None:
                continue
            need = e.plan_energy(plan.steps, self.payload) + self._station_energy(pad)
            if self.cfg.battery_policy == "predictive" and need + e.reserve > self.battery:
                energy_short = True
                continue
            self.post_pad = pad
            self.station_sid = self.world.stations.index(pad) if is_station else None
            self.state = DroneState.TO_CUSTOMER
            self._commit(t, plan)
            return
        # nothing feasible from here
        if self.airborne:
            self._hold(t)
        elif energy_short and self.site in self.world.hubs:
            # still at the hub: put the parcel back on the shelf and go recharge
            self.carrying = False
            self._drop_task(t, "insufficient battery for delivery")
            self.state = DroneState.IDLE
            self._go_swap(t)
        # otherwise stay on the ground (safe) and retry next tick

    def _post_delivery_pads(self, dest: Cell, battery_after: float) -> list[tuple[Site, bool]]:
        """Where to land after the drop at cell ``dest``, best first."""
        hub = self._nearest_hub(dest)
        if self.cfg.battery_policy == "naive":
            if battery_after < self.cfg.naive_threshold * self.cfg.battery_capacity:
                return [(self._choose_station(dest, battery_after), True)]
            return [(hub, False)]
        home_cost = self.energy.estimate(self.world.dist(dest, hub), slack=self.cfg.detour_factor, takeoff=False)
        stations = sorted(self.world.stations, key=lambda s: self.world.dist(dest, s))
        best_station = self._choose_station(dest, battery_after)
        st_opts = [(best_station, True)] + [(s, True) for s in stations if s != best_station]
        if battery_after - home_cost < self.cfg.swap_threshold * self.cfg.battery_capacity:
            return st_opts + [(hub, False)]
        return [(hub, False)] + st_opts

    def _choose_station(self, frm, battery: float) -> Site:
        """Pick the station minimising travel + believed queueing delay (must be reachable)."""
        best, best_cost = None, math.inf
        for sid, cell in enumerate(self.world.stations):
            d = self.world.dist(frm, cell)
            need = self.energy.estimate(d, slack=self.cfg.detour_factor, takeoff=not self.airborne)
            if need > battery and best is not None:
                continue
            b = self.station_beliefs.get(sid, {})
            wait = b.get("est_wait_now", 0) + b.get("expected", 0) * self.cfg.swap_ticks / self.cfg.station_bays
            cost = d + max(0, wait - d) + (1000 if need > battery else 0)
            if cost < best_cost:
                best, best_cost = cell, cost
        return best

    def _go_swap(self, t: int, target: Site | None = None, reason: int = TRIP_LOW) -> bool:
        target = target or self._choose_station(self.pos, self.battery)
        self.station_sid = self.world.stations.index(target)
        self.post_pad = target
        self.trip_reason = reason
        if self.site == target and not self.airborne:
            self.state = DroneState.QUEUED
            self.send(f"station{self.station_sid}", Performative.REQUEST, t, type="swap", pack=self.pack)
            return True
        prev = self.state
        self.state = DroneState.TO_STATION
        if not self._replan(t):
            self.state = prev if not self.airborne else DroneState.TO_STATION
            return False
        eta = self.plan.end_t
        self.send(f"station{self.station_sid}", Performative.REQUEST, t, type="reserve_swap", eta=eta)
        if reason == TRIP_BEFORE_JOB:
            self._log(t, f"heads to Station {self.station_sid} for a fresh battery before its next job")
        else:
            self._log(t, f"is low on battery ({self.soc:.0%}) and heads to Station {self.station_sid} for a swap")
        return True

    def _battery_emergency(self, t: int) -> bool:
        """Divert to the nearest station if the remaining plan would breach the critical reserve."""
        if self.state == DroneState.TO_STATION or self._in_drop(t):
            return False
        if self.cfg.battery_policy == "predictive":
            if self.plan is None:           # holding in the air while blocked
                remaining = self._station_energy(self.pos, takeoff=False)
            else:
                remaining = self.energy.plan_energy(self.plan.remaining(t), self.payload)
                remaining += self._station_energy(self.plan.steps[-1].cell)
            trigger = self.battery - remaining < self.energy.critical
        else:
            trigger = self.soc < self.cfg.critical_fraction * 2
        if not trigger:
            return False
        self.stats["emergencies"] += 1
        nearest = min(self.world.stations, key=lambda s: (self.world.dist(self.pos, s), s))
        self.station_sid = self.world.stations.index(nearest)
        self.trip_reason = TRIP_EMERGENCY
        self._log(t, f"BATTERY EMERGENCY: not enough charge left to finish safely ({self.soc:.0%}), "
                     f"diverting to Station {self.station_sid}"
                     + (" with the parcel still on board" if self.carrying else ""))
        self.post_pad = nearest
        self.state = DroneState.TO_STATION
        self.needs_replan = "emergency"
        self._replan(t)
        if self.plan:
            self.send(f"station{self.station_sid}", Performative.REQUEST, t, type="reserve_swap",
                      eta=self.plan.end_t)
        return True

    # ================================================================ planning
    def _waypoints(self, t: int) -> list[Waypoint] | None:
        st = self.state
        if st == DroneState.TO_PICKUP:
            return [Waypoint(self.task["hub"], "land")]
        if st == DroneState.TO_CUSTOMER:
            return [Waypoint(self.task["dest"], "drop", self.cfg.drop_ticks), Waypoint(self.post_pad, "land")]
        if st in (DroneState.RETURNING, DroneState.TO_STATION):
            return [Waypoint(self.post_pad, "land")]
        return None

    def _make_plan(self, t: int, wps: list[Waypoint], ignore: frozenset = frozenset()) -> Plan | None:
        extra = None
        if self.cfg.coordination == "reactive" and self.sensor is not None:
            extra = self.sensor(self, t)
        return self.planner.plan(self.did, self.pos, t, self.airborne, wps, now=t, ignore=ignore, extra_blocked=extra)

    def _replan(self, t: int) -> bool:
        wps = self._waypoints(t)
        if wps is None:
            return False
        if self._in_drop(t):
            self.needs_replan = None
            return True
        self.res.release(self.did, t)
        ignore = frozenset()
        if self.hold_streak >= self.cfg.hold_escalation and self.cfg.coordination == "cooperative":
            ignore = self._escalation_ignore_set(t)
        plan = None
        if self.cfg.coordination == "reactive" and self.airborne and self.deviation_streak >= 4:
            plan = self._sidestep(t, wps)
        if plan is None:
            plan = self._make_plan(t, wps, ignore)
        if plan is None:
            self.stats["plan_failures"] += 1
            if self.airborne:
                self._hold(t)
            return False
        self.stats["replans"] += 1
        if ignore:
            self.stats["escalations"] += 1
            self._log(t, f"has been blocked for {self.hold_streak} ticks and claims right of way")
        self._commit(t, plan)
        return True

    def _sidestep(self, t: int, wps: list[Waypoint]) -> Plan | None:
        """Reactive deadlock breaker: hop to a random free neighbour, then re-plan.

        Without reservations two drones can block each other indefinitely
        (e.g. head-on in a corridor, each re-planning the same detour). A random
        sidestep breaks the symmetry - the standard trick of reactive schemes.
        """
        sensed = self.sensor(self, t) if self.sensor else set()
        taken = {c for c, tt in sensed if tt == t + 1}
        options = [n for n in self.world.neighbors(self.pos)
                   if n not in taken and not self.world.in_active_nfz(n, t + 1)]
        if not options:
            return None
        n = self.rng.choice(options)
        rest = self.planner.plan(self.did, n, t + 1, True, wps, now=t, extra_blocked=sensed)
        if rest is None:
            return None
        return Plan([PlanStep(t, self.pos, True)] + rest.steps, rest.waypoints)

    def _escalation_ignore_set(self, t: int) -> frozenset:
        """Plan through the claims of lower-ranked drones (they will be asked to yield).

        Drones hovering over a customer are never overridden: they cannot move.
        The strict total order of ``rank`` guarantees the top drone always wins,
        so escalation cannot livelock.
        """
        self.escalated = True
        return frozenset(o.did for o in self.peers if o.did != self.did and o.alive
                         and o.rank() < self.rank() and not o._in_drop(t))

    def _commit(self, t: int, plan: Plan) -> None:
        assert plan.start_t == t and plan.is_contiguous(), "plans must start now and have one step per tick"
        self.res.release(self.did, t)
        bumped = self.res.reserve_path(self.did, plan.airborne_states())
        for other in bumped - {self.did}:
            self.send(f"drone{other}", Performative.REQUEST, t, type="yield", by=self.did)
        self.plan = plan
        self.needs_replan = None
        self.hold_streak = 0
        self.escalated = False
        self.plan_log.append((t, [list(s.cell) for s in plan.steps]))

    def _hold(self, t: int) -> None:
        """No plan found in the air: hover, claim the cell, ask intruders to yield."""
        self.plan = None
        self.hold_streak += 1
        self.stats["holds"] += 1
        self.res.release(self.did, t)
        bumped = self.res.hold(self.did, self.pos, t, t + self.cfg.reservation_hold)
        for other in bumped - {self.did}:
            self.send(f"drone{other}", Performative.REQUEST, t, type="yield", by=self.did)
        self.needs_replan = "blocked"

    # =============================================================== execution
    def intent(self, t: int) -> tuple[Cell, bool]:
        """Where this drone wants to be at t+1 (cell, airborne)."""
        if not self.alive:
            return self.pos, False
        if self.plan is not None:
            nxt = self.plan.step_at(t + 1)
            cur = self.plan.step_at(t)
            if nxt is not None and cur is not None and cur.cell == self.pos and cur.airborne == self.airborne:
                return nxt.cell, nxt.airborne
            if cur is not None and cur.tag == "land" and cur.cell == self.pos and self.airborne:
                return ground(self.pos), False
        return self.pos, self.airborne

    def apply_motion(self, t_new: int, cell: Cell, airborne: bool) -> None:
        """Physics: move, burn energy, detect depletion."""
        cost = self.energy.transition(self.airborne, self.pos, airborne, cell, self.payload)
        if airborne:
            self.stats["airborne_ticks"] += 1
            if cell[2] > 1:
                self.stats["upper_layer_ticks"] += 1
            if self.world.building_height(cell):
                self.stats["over_building_ticks"] += 1
                if cell[2] == self.world.n_layers:
                    self.stats["over_building_top_ticks"] += 1
            if self.airborne and cell != self.pos:
                if cell[0] == self.pos[0] and cell[1] == self.pos[1]:
                    self.stats["climbs" if cell[2] > self.pos[2] else "descents"] += 1
                else:
                    self.stats["cells_flown"] += 1
            elif self.airborne:
                self.stats["hover_ticks"] += 1
        self.pack.charge = max(0.0, self.pack.charge - cost)
        self.stats["energy_used"] += cost
        self.pos = cell if airborne else ground(cell)
        if airborne and self.pack.charge <= 0.0:
            self._die(t_new)

    def _die(self, t: int) -> None:
        self._log(t, "!! RAN OUT OF BATTERY IN FLIGHT and is lost"
                     + (f" with order #{self.task['oid']}" if self.task and self.carrying else ""))
        self.state = DroneState.DEAD
        self.pos = ground(self.pos)         # it falls out of the airspace
        self.plan = None
        self.res.release(self.did)
        if self.task:
            self.send("dispatcher", Performative.INFORM, t, type="package_lost" if self.carrying else "task_dropped",
                      oid=self.task["oid"])
            if not self.carrying:
                self.send("dispatcher", Performative.FAILURE, t, type="task", oid=self.task["oid"], reason="drone lost")
        self.task = None

    def after_move(self, t: int) -> None:
        """React to arrival events and detect deviations from the committed plan."""
        if not self.alive or self.plan is None:
            return
        step = self.plan.step_at(t)
        on_plan = step is not None and step.cell == self.pos and step.airborne == self.airborne
        if not on_plan:
            if self.state in AIR_STATES or self.airborne:
                self.stats["deviations"] += 1
                self.deviation_streak += 1
                if not self._repair(t):
                    self.res.release(self.did, t)
                    if self.airborne:
                        bumped = self.res.hold(self.did, self.pos, t, t + 1)
                        for other in bumped - {self.did}:
                            self.send(f"drone{other}", Performative.REQUEST, t, type="yield", by=self.did)
                    self.plan = None
                    self.needs_replan = "deviation"
            return
        self.deviation_streak = 0
        tag = step.tag
        if tag == "drop_done" and self.carrying:
            oid = self.task["oid"]
            self.carrying = False
            self.stats["deliveries"] += 1
            self.send("dispatcher", Performative.INFORM, t, type="delivered", oid=oid)
            self.task = None
            if self.station_sid is not None and self.post_pad in self.world.stations:
                self.state = DroneState.TO_STATION
                self.trip_reason = TRIP_AFTER_DELIVERY
                self._log(t, f"heads to Station {self.station_sid} to swap its battery ({self.soc:.0%} left)")
                self.send(f"station{self.station_sid}", Performative.REQUEST, t, type="reserve_swap",
                          eta=self.plan.end_t)
            else:
                self.state = DroneState.RETURNING
        elif tag == "land":
            self.pos = ground(self.pos)     # touch down on the pad
            self.plan = None
            self.res.release(self.did, t + 1)
            if self.state == DroneState.TO_PICKUP:
                self.state = DroneState.LOADING
                self.load_until = t + self.cfg.loading_ticks
            elif self.state == DroneState.TO_STATION:
                sid = self.world.stations.index(self.site)
                self.station_sid = sid
                self.state = DroneState.QUEUED
                self.send(f"station{sid}", Performative.REQUEST, t, type="swap", pack=self.pack)
                self._log(t, f"lands at Station {sid} with {self.soc:.0%} battery and waits for a swap")
            else:
                self.state = DroneState.IDLE
                self.idle_since = t

    def _repair(self, t: int) -> bool:
        """Cheap plan repair: if we merely lost one tick, shift the rest of the plan by +1."""
        if self.cfg.coordination == "reactive" and self.deviation_streak > 2:
            return False    # repeatedly blocked: re-plan around what we sense instead
        prev = self.plan.step_at(t - 1)
        if prev is None or prev.cell != self.pos or prev.airborne != self.airborne:
            return False
        # the step we are still at (t-1) becomes the step at t, and so on
        rest = self.plan.steps[t - 1 - self.plan.start_t:]
        shifted = [PlanStep(s.t + 1, s.cell, s.airborne, s.tag) for s in rest]
        zones = self.world.known_zones(t)
        for a, b in zip(shifted, shifted[1:]):
            if not b.airborne:
                continue
            if a.airborne:
                if not self.res.move_free(a.cell, b.cell, a.t, self.did):
                    return False
            elif not self.res.vertex_free(b.cell, b.t, self.did):
                return False
            if any(z.start_t <= b.t < z.end_t and z.contains(b.cell) for z in zones):
                return False
        drop = next((s for s in shifted if s.tag == "drop_start"), None)
        if drop is not None:
            done = next(s for s in shifted if s.tag == "drop_done")
            if not self.res.span_free(drop.cell, drop.t, done.t, self.did):
                return False
        new = Plan(shifted, self.plan.waypoints)
        if self.cfg.battery_policy == "predictive":
            need = self.energy.plan_energy(new.steps, self.payload) + self._station_energy(new.steps[-1].cell)
            if need + self.energy.critical > self.battery:
                return False
        self.stats["repairs"] += 1
        self._commit(t, new)
        return True
