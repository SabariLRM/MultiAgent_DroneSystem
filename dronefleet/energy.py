"""Battery packs and the flight-energy model.

Energy per tick depends on what the drone does and how much it carries:

    move   = move_cost    * (1 + payload_factor * kg)    one cell horizontally
    hover  = hover_cost   * (1 + payload_factor * kg)
    takeoff= takeoff_cost * (1 + payload_factor * kg)    ground -> layer 1
    climb  = climb_cost   * (1 + payload_factor * kg)    layer z -> z + 1
    descend= descend_cost * (1 + payload_factor * kg)    layer z -> z - 1 (z >= 2)

Touch-down (layer 1 -> ground) and waiting on the ground are free. Packs are
physical objects: a swap hands the drone's depleted pack to the station and
takes a charged one, so the number of packs in the system is conserved and
chargers can become a bottleneck.

Fast estimates (used when bidding and to price "reach a station afterwards")
multiply a flight distance in ticks by :func:`step_cost_bound`, the most a
tick of flight can cost on a route that does not end higher than it starts.
Every route the agents estimate ends at layer 1 (a drop or a landing), so the
estimate never undercounts the energy of a shortest route. With customers in
buildings a drop can be higher (above the roof); the leg after it descends
as much again, so a bid, which sums hub -> drop -> station, is still never
undercounted.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


def step_cost_bound(cfg) -> float:
    """Upper bound on the average energy per tick of flight (no payload).

    A route from layer ``z_a`` down to layer ``z_b <= z_a`` with ``H``
    horizontal moves, ``C`` climbs and ``D >= C`` descents costs
    ``H*move + C*(climb + descend) + (D - C)*descend``, which is at most
    ``(H + C + D) * max(move, (climb + descend) / 2, descend)``. With one
    layer there are no climbs or descents, so the bound is ``move_cost``.
    """
    if cfg.n_layers == 1:
        return cfg.move_cost
    return max(cfg.move_cost, (cfg.climb_cost + cfg.descend_cost) / 2, cfg.descend_cost)


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
        self.step_bound = step_cost_bound(cfg)

    def _f(self, payload: float) -> float:
        return 1.0 + self.cfg.payload_factor * payload

    def move(self, payload: float = 0.0) -> float:
        return self.cfg.move_cost * self._f(payload)

    def hover(self, payload: float = 0.0) -> float:
        return self.cfg.hover_cost * self._f(payload)

    def takeoff(self, payload: float = 0.0) -> float:
        return self.cfg.takeoff_cost * self._f(payload)

    def climb(self, payload: float = 0.0) -> float:
        return self.cfg.climb_cost * self._f(payload)

    def descend(self, payload: float = 0.0) -> float:
        return self.cfg.descend_cost * self._f(payload)

    def step(self, payload: float = 0.0) -> float:
        """Upper bound on one tick of flight, for estimates (see :func:`step_cost_bound`)."""
        return self.step_bound * self._f(payload)

    def transition(self, prev_air: bool, prev_cell, now_air: bool, now_cell, payload: float) -> float:
        """Energy for one tick of actual (or planned) motion."""
        if not now_air:
            return 0.0                      # on the ground / just touched down
        if not prev_air:
            return self.takeoff(payload)
        if prev_cell == now_cell:
            return self.hover(payload)
        if prev_cell[0] == now_cell[0] and prev_cell[1] == now_cell[1]:
            return self.climb(payload) if now_cell[2] > prev_cell[2] else self.descend(payload)
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
        """Fast estimate from a flight distance in ticks (BFS), used when bidding."""
        e = cells * slack * self.step(payload) + hover_ticks * self.hover(payload)
        if takeoff:
            e += self.takeoff(payload)
        return e

    @property
    def reserve(self) -> float:
        return self.cfg.reserve_fraction * self.capacity

    @property
    def critical(self) -> float:
        return self.cfg.critical_fraction * self.capacity
