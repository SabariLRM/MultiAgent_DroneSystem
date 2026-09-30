"""Discrete-time simulation engine.

One tick, in order:

1. **Environment** - new orders arrive (Poisson); NFZ announcements (area,
   time window and closed layers) are broadcast by air-traffic control.
2. **Dispatcher** - closes the previous auction round / opens a new one.
3. **Stations** - charge packs, progress swaps, broadcast status.
4. **Drones deliberate** (highest right-of-way first) - read messages, bid,
   accept awards, (re)plan and reserve space-time paths.
5. **Yield rounds** - drones whose reservations were overridden re-plan in
   the same tick.
6. **Intents -> disturbances -> reactive resolution** - wind gusts may hold a
   drone back; the right-of-way rule removes any remaining conflict.
7. **Physics** - positions (x, y, layer) and batteries update; collisions
   are *detected* independently of how they were avoided.
8. **Drones observe** - arrivals, drops, landings, deviations from plan.

With ``motion = "continuous"`` steps 6-7 are replaced by the tactical layer
and continuous physics (:mod:`dronefleet.flight`): 20 sub-steps of path
following, 3-D ORCA, wind and a power model, after which each drone's
position is reported back to its agent as a grid cell.
"""

from __future__ import annotations

import random
import time

from .agents.dispatcher import DispatcherAgent
from .agents.drone import AIR_STATES, DroneAgent, DroneState
from .agents.station import SwapStationAgent
from .config import SimConfig
from .energy import BatteryPack, EnergyModel
from .flight import FlightLayer
from .messages import Message, MessageBus, Performative
from .metrics import compute_metrics
from .orders import OrderGenerator
from .planner import SpaceTimePlanner
from .power import ContinuousEnergyModel
from .replay import TraceRecorder
from .reservation import ReservationTable
from .traffic import detect_collisions, resolve
from .wind import WindField
from .world import generate_world, manhattan

SENSOR_RANGE = 3


class Simulation:
    def __init__(self, cfg: SimConfig | None = None):
        self.cfg = cfg = (cfg or SimConfig()).validate()
        self.world = generate_world(cfg)
        self.bus = MessageBus()
        self.res = ReservationTable(enabled=cfg.coordination == "cooperative")
        self.continuous = cfg.motion == "continuous"
        if self.continuous:
            self.wind = WindField(cfg)
            self.planner = SpaceTimePlanner(self.world, self.res, cfg.max_expansions,
                                            land_dwell=cfg.land_ticks)
            self.energy = ContinuousEnergyModel(cfg, self.wind)
        else:
            self.wind = None
            self.planner = SpaceTimePlanner(self.world, self.res, cfg.max_expansions)
            self.energy = EnergyModel(cfg)
        self.dispatcher = DispatcherAgent(cfg, self.bus)
        self.orders = OrderGenerator(cfg, self.world, self.energy if self.continuous else None)
        rng = random.Random(cfg.seed * 13 + 1)
        self.gust_rng = random.Random(cfg.seed * 31 + 5)

        pid = 0
        self.stations: list[SwapStationAgent] = []
        for sid, cell in enumerate(self.world.stations):
            packs = []
            for _ in range(cfg.station_spare_packs):
                pid += 1
                packs.append(BatteryPack(pid, cfg.battery_capacity, cfg.battery_capacity))
            self.stations.append(SwapStationAgent(sid, cell, cfg, self.bus, packs))

        self.drones: list[DroneAgent] = []
        for did in range(cfg.n_drones):
            pid += 1
            soc = rng.uniform(0.55, 1.0)
            pack = BatteryPack(pid, cfg.battery_capacity, round(cfg.battery_capacity * soc, 1))
            start = self.world.hubs[did % len(self.world.hubs)]
            d = DroneAgent(did, cfg, self.bus, self.world, self.planner, self.res, self.energy,
                           start, pack, sensor=self._sense)
            self.drones.append(d)
        for d in self.drones:
            d.peers = self.drones

        self.t = 0
        self.last_gusted: set[int] = set()     # held back by wind in the last move
        self.last_forced: set[int] = set()     # gave way under the right-of-way rule
        self.collisions: list[tuple] = []
        self.gusts = 0
        self.forced_holds = 0
        self.nfz_violations = 0
        self.events: list[tuple[int, str]] = []
        self.wall_time = 0.0
        self.flight = FlightLayer(self) if self.continuous else None
        self.trace = TraceRecorder(self) if cfg.record_trace else None

    # ------------------------------------------------------------- sensing
    def _sense(self, drone: DroneAgent, t: int) -> set:
        """Local perception for the reactive baseline: nearby drones (horizontal range,
        any layer), assumed to stay put."""
        seen = set()
        for o in self.drones:
            if o is drone or not o.airborne or manhattan(o.pos, drone.pos) > SENSOR_RANGE:
                continue
            for k in range(1, 4):
                seen.add((o.pos, t + k))
        return seen

    # --------------------------------------------------------------- tick
    def step(self) -> None:
        t = self.t
        ev: list[str] = []

        for o in self.orders.tick(t):
            self.dispatcher.add_order(o)
            ev.append(f"New order #{o.oid}{' (EXPRESS)' if o.express else ''}: a {o.weight:.1f} kg parcel "
                      f"from Hub {self.world.hubs.index(o.hub)} to a customer, due by t={o.deadline_t}")
        for z in self.world.zones_announced_at(t):
            layers = z.layers(self.world.n_layers)
            self.bus.send(Message("atc", "drones", Performative.INFORM,
                                  {"type": "nfz", "rect": z.rect, "start_t": z.start_t, "end_t": z.end_t,
                                   "layers": layers}, t))
            closed = "" if layers == (1, self.world.n_layers) else f" (layers {layers[0]}-{layers[1]} only)"
            ev.append(f"Air traffic control announces a no-fly zone{closed}, closed from t={z.start_t} to t={z.end_t}")

        ev += self.dispatcher.step(t)
        for s in self.stations:
            ev += s.step(t)

        alive = [d for d in self.drones if d.alive]
        for d in sorted(alive, key=lambda d: d.rank(), reverse=True):
            d.deliberate(t)
        for _ in range(4):                      # same-tick yield negotiation
            pending = [d for d in alive if d.inbox]
            if not pending:
                break
            for d in pending:
                d.process_urgent(t)

        if self.flight is not None:
            self._move_continuous(t, alive, ev)
        else:
            self._move_grid(t, alive, ev)
        for d in alive:
            if d.airborne and self.world.in_active_nfz(d.pos, self.t):
                self.nfz_violations += 1

        for d in alive:
            d.after_move(self.t)
        if self.flight is not None:
            self.flight.after_agents(self.t)
        for d in self.drones:
            if d.event_log:
                ev += d.event_log
                d.event_log.clear()

        self.res.prune(self.t - 1)
        self.events += [(t, e) for e in ev]
        if self.trace:
            self.trace.record(self.t, ev)

    def _move_grid(self, t: int, alive: list, ev: list) -> None:
        """Intents -> wind gusts -> right-of-way resolution -> cell moves -> collision detector."""
        intents = {}
        for d in alive:
            tc, ta = d.intent(t)
            intents[d.did] = (d.pos, d.airborne, tc, ta)
        gusted = set()
        for i, (cc, ca, tc, ta) in list(intents.items()):
            if ca and ta and tc != cc and self.gust_rng.random() < self.cfg.gust_prob:
                intents[i] = (cc, ca, cc, ca)
                gusted.add(i)
        self.gusts += len(gusted)
        self.last_gusted = gusted
        if self.cfg.coordination != "none":
            final, forced = resolve(intents, {d.did: d.rank() for d in alive})
            self.forced_holds += len(forced)
        else:
            final, forced = intents, set()
        self.last_forced = forced

        before = {i: (v[0], v[1]) for i, v in intents.items()}
        for d in alive:
            _, _, tc, ta = final[d.did]
            d.apply_motion(t + 1, tc, ta)
        self.t = t + 1

        after = {d.did: (d.pos, d.airborne) for d in alive}
        for kind, ids, cell in detect_collisions(before, after):
            self.collisions.append((self.t, kind, ids, cell))
            what = "are in the same cell" if kind == "vertex" else "fly through each other head-on"
            ev.append(f"!! COLLISION: drones {' and '.join(map(str, ids))} {what} "
                      f"at ({cell[0]}, {cell[1]}) on layer {cell[2]}")

    def _move_continuous(self, t: int, alive: list, ev: list) -> None:
        """Physics sub-steps (path following, ORCA, wind, energy), then report cells to the agents."""
        self.flight.advance(t)
        self.t = t + 1
        for d in alive:
            self.flight.observe(d, self.t)
        self.last_gusted = set()
        self.last_forced = self.flight.orca_drones
        ev += self.flight.events
        self.flight.events = []

    # ---------------------------------------------------------------- run
    def done(self) -> bool:
        if self.t <= self.cfg.order_until:
            return False
        if any(o.status not in ("delivered", "failed") for o in self.dispatcher.orders.values()):
            return False
        return all(not d.airborne and d.state not in AIR_STATES and d.state != DroneState.LOADING
                   for d in self.drones)

    def run(self, progress: bool = False) -> dict:
        start = time.perf_counter()
        while self.t < self.cfg.max_ticks and not self.done():
            self.step()
            if progress and self.t % 100 == 0:
                m = self.dispatcher.orders.values()
                done = sum(o.status == "delivered" for o in m)
                print(f"  t={self.t:4d}  orders {done}/{len(m)} delivered  "
                      f"collisions={len(self.collisions)}", flush=True)
        self.wall_time = time.perf_counter() - start
        return compute_metrics(self)
