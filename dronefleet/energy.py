"""Battery packs and the flight-energy model.

Energy per tick depends on what the drone does and how much it carries:

    move   = move_cost   * (1 + payload_factor * kg)
    hover  = hover_cost  * (1 + payload_factor * kg)
    takeoff= takeoff_cost* (1 + payload_factor * kg)

Landing and waiting on the ground are free. Packs are physical objects: a swap
hands the drone's depleted pack to the station and takes a charged one, so the
number of packs in the system is conserved and chargers can become a
bottleneck.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass
class BatteryPack:
    pid: int
    capacity: float
    charge: float
    cycles: int = 0

    @property
    def soc(self) -> float:
        return self.charge / self.capacity


class EnergyModel:
    def __init__(self, cfg):
        self.cfg = cfg
        self.capacity = cfg.battery_capacity

    def _f(self, payload: float) -> float:
        return 1.0 + self.cfg.payload_factor * payload

    def move(self, payload: float = 0.0) -> float:
        return self.cfg.move_cost * self._f(payload)

    def hover(self, payload: float = 0.0) -> float:
        return self.cfg.hover_cost * self._f(payload)

    def takeoff(self, payload: float = 0.0) -> float:
        return self.cfg.takeoff_cost * self._f(payload)

    def transition(self, prev_air: bool, prev_cell, now_air: bool, now_cell, payload: float) -> float:
        """Energy for one tick of actual (or planned) motion."""
        if not now_air:
            return 0.0                      # on the ground / just landed
        if not prev_air:
            return self.takeoff(payload)
        if prev_cell == now_cell:
            return self.hover(payload)
        return self.move(payload)

    def plan_energy(self, steps: Iterable, payload: float) -> float:
        """Energy of a sequence of ``PlanStep`` s; payload is released at ``drop_done``."""
        total = 0.0
        prev = None
        for s in steps:
            if prev is not None:
                total += self.transition(prev.airborne, prev.cell, s.airborne, s.cell, payload)
            if s.tag == "drop_done":
                payload = 0.0
            prev = s
        return total

    def estimate(self, cells: float, payload: float = 0.0, takeoff: bool = True,
                 hover_ticks: int = 0, slack: float = 1.0) -> float:
        """Fast straight-line (BFS distance) estimate used when bidding."""
        e = cells * slack * self.move(payload) + hover_ticks * self.hover(payload)
        if takeoff:
            e += self.takeoff(payload)
        return e

    @property
    def reserve(self) -> float:
        return self.cfg.reserve_fraction * self.capacity

    @property
    def critical(self) -> float:
        return self.cfg.critical_fraction * self.capacity
