"""Battery-swap station agent.

A station owns a small inventory of battery packs and ``bays`` robotic swap
arms. Drones that land are served first-come-first-served; a swap needs a
free bay *and* a pack charged to at least ``READY_SOC``. Depleted packs taken
out of drones go on the chargers.

Each tick the station broadcasts its status (queue length, charged packs and an
expected-wait estimate). Drones use these broadcasts as *beliefs* when choosing
where to swap, and send a reservation REQUEST with their ETA so the station's
estimate accounts for drones that are already on their way.
"""

from __future__ import annotations

import math
from collections import deque

from ..energy import BatteryPack
from ..messages import Performative
from ..world import Cell
from .base import Agent

READY_SOC = 0.95


class SwapStationAgent(Agent):
    role = "stations"

    def __init__(self, sid: int, cell: Cell, cfg, bus, packs: list[BatteryPack]):
        super().__init__(f"station{sid}", bus)
        self.sid = sid
        self.cell = cell
        self.cfg = cfg
        self.packs: list[BatteryPack] = packs
        self.queue: deque[tuple[str, BatteryPack, int]] = deque()   # (drone, pack, arrival_t)
        self.active: dict[str, tuple[int, BatteryPack, BatteryPack]] = {}  # drone -> (finish_t, new, old)
        self.expected: dict[str, int] = {}                          # drone -> eta
        self.swaps_done = 0
        self.wait_ticks_total = 0
        self.max_queue = 0

    # ---------------------------------------------------------------- beliefs
    def charged_packs(self) -> int:
        return sum(1 for p in self.packs if p.soc >= READY_SOC)

    def estimate_wait(self, eta: int, now: int) -> int:
        """Expected ticks a drone arriving at ``eta`` waits before its swap starts."""
        ahead = len(self.queue) + len(self.active)
        ahead += sum(1 for d, e in self.expected.items() if e <= eta)
        bays = self.cfg.station_bays
        # bay contention
        busy_until = 0
        if ahead >= bays:
            rounds = ahead // bays
            busy_until = rounds * self.cfg.swap_ticks
        # pack availability: the k-th drone ahead needs the k-th best pack
        need = ahead + 1
        socs = sorted((p.charge for p in self.packs), reverse=True)
        pack_wait = 0
        if len(socs) >= need:
            deficit = self.cfg.battery_capacity * READY_SOC - socs[need - 1]
            pack_wait = max(0, math.ceil(deficit / self.cfg.charge_rate))
        else:
            # waits for a depleted pack from a drone ahead to recharge
            pack_wait = math.ceil(self.cfg.battery_capacity * 0.8 / self.cfg.charge_rate)
        travel = max(0, eta - now)
        return max(0, max(busy_until, pack_wait) - travel)

    # ------------------------------------------------------------------- step
    def step(self, t: int) -> list[str]:
        """Returns human-readable events for the trace."""
        events = []
        for msg in self.drain():
            kind = msg.content.get("type")
            if msg.performative == Performative.REQUEST and kind == "reserve_swap":
                eta = msg.content["eta"]
                self.expected[msg.sender] = eta
                self.send(msg.sender, Performative.AGREE, t, type="swap_reserved",
                          station=self.sid, est_wait=self.estimate_wait(eta, t))
            elif msg.performative == Performative.CANCEL and kind == "reserve_swap":
                self.expected.pop(msg.sender, None)
            elif msg.performative == Performative.REQUEST and kind == "swap":
                self.expected.pop(msg.sender, None)
                pack: BatteryPack = msg.content["pack"]
                self.queue.append((msg.sender, pack, t))
                self.max_queue = max(self.max_queue, len(self.queue))

        # charge idle packs
        for p in self.packs:
            if p.charge < p.capacity:
                p.charge = min(p.capacity, p.charge + self.cfg.charge_rate)

        # finish swaps
        for drone, (finish_t, new_pack, old_pack) in list(self.active.items()):
            if t >= finish_t:
                del self.active[drone]
                old_pack.cycles += 1
                self.packs.append(old_pack)          # depleted pack goes on the charger
                self.swaps_done += 1
                self.send(drone, Performative.INFORM, t, type="swap_done", station=self.sid, pack=new_pack)

        # start swaps (FIFO) while bays and charged packs are available
        while self.queue and len(self.active) < self.cfg.station_bays:
            ready = [p for p in self.packs if p.soc >= READY_SOC]
            if not ready:
                break
            best = max(ready, key=lambda p: p.charge)
            drone, old_pack, arr_t = self.queue.popleft()
            self.packs.remove(best)
            self.active[drone] = (t + self.cfg.swap_ticks, best, old_pack)
            self.wait_ticks_total += t - arr_t
            self.send(drone, Performative.INFORM, t, type="swap_started", station=self.sid)
            events.append(f"Station {self.sid} starts swapping Drone {drone.removeprefix('drone')}'s battery "
                          f"({old_pack.soc:.0%} left)")

        self.send("drones", Performative.INFORM, t, type="station_status", station=self.sid,
                  cell=self.cell, queue=len(self.queue), active=len(self.active),
                  charged=self.charged_packs(), expected=len(self.expected),
                  est_wait_now=self.estimate_wait(t, t))
        return events

    def snapshot(self) -> list:
        return [len(self.queue), len(self.active), self.charged_packs(), len(self.packs)]
