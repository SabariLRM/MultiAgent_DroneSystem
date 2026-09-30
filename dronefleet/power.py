"""Continuous-flight power model and the energy model the agents plan with.

Physics (energy units per second)::

    P = hover_power * (m / m0) ** 1.5  +  drag_power * |v_air| ** 3  +  climb_power * m * g * v_z

with ``m = m0 + payload`` and ``v_air`` the drone's velocity relative to the
air. Descending (``v_z < 0``) lets gravity do some of the work; the total is
never below ``min_power_fraction`` of the hover power. With the defaults one
100 m cell at the 10 m/s cruise speed in still air costs
``(0.07 + 3e-5 * 10**3) * 10 s = 1.0`` unit, hovering a tick 0.7, climbing a
layer 1.2 and descending one about 0.55, the same scale as the grid model
(:mod:`dronefleet.energy`), so every battery threshold, bid estimate and the
predictive invariant keep their meaning.

:class:`ContinuousEnergyModel` is the drop-in replacement for
:class:`~dronefleet.energy.EnergyModel` in continuous mode. It prices the
same actions from the power model and the *forecast* wind (mean wind at the
flight layer plus the RMS gust), so a planned leg into the wind costs more
than one with the wind behind it. Fast estimates, which do not know the
route's direction, use the price averaged over all headings.
"""

from __future__ import annotations

import math

from .energy import EnergyModel

G = 9.81


class PowerModel:
    def __init__(self, cfg):
        self.m0 = cfg.base_mass_kg
        self.ph = cfg.hover_power
        self.cd = cfg.drag_power
        self.kc = cfg.climb_power
        self.floor = cfg.min_power_fraction

    def hover_factor(self, payload: float) -> float:
        """Hover power relative to the empty drone: (m / m0) ** 1.5."""
        return ((self.m0 + payload) / self.m0) ** 1.5

    def power(self, payload: float, airspeed: float, vz: float, hf: float | None = None) -> float:
        """Electrical power (energy units per second) at a given airspeed and climb rate."""
        if hf is None:
            hf = self.hover_factor(payload)
        h = self.ph * hf
        p = h + self.cd * airspeed * airspeed * airspeed + self.kc * (self.m0 + payload) * G * vz
        f = self.floor * h
        return p if p > f else f


class ContinuousEnergyModel(EnergyModel):
    """Planning energy from the power model and the wind forecast (continuous mode).

    A planned horizontal cell is flown at the cruise speed ``cell_m / tick_s``
    unless the wind makes that impossible: the drone's airspeed may not exceed
    ``max_airspeed``, and a gust margin of one RMS gust is kept in reserve,
    so straight into a strong wind the drone is slower and the cell takes
    longer (and costs more). Squared airspeeds are averaged over the gusts
    (``|v - mean|^2 + gust^2``). Every price is multiplied by
    ``energy_margin`` to cover what the plan does not model: catching up
    after a delay, avoidance manoeuvres, acceleration.
    """

    def __init__(self, cfg, wind):
        super().__init__(cfg)
        self.pm = PowerModel(cfg)
        self.wind = wind
        self.tick = cfg.tick_s
        self.cell_m = cfg.cell_m
        self.cruise = cfg.cell_m / cfg.tick_s
        self.layer_m = cfg.layer_m
        self.vair = cfg.max_airspeed
        self.margin = cfg.energy_margin
        self.climb_rate = cfg.max_climb
        self.descent_rate = cfg.max_descent
        self.t_climb = cfg.layer_m / cfg.max_climb
        self.t_descend = cfg.layer_m / cfg.max_descent
        # a take-off also accelerates to and from the climb rate
        self.t_takeoff = self.t_climb + cfg.max_climb / cfg.max_accel
        self._cell: dict[tuple, tuple[float, float]] = {}
        self._still: dict[tuple, float] = {}
        # heading-averaged cell (fast estimates do not know the route's direction)
        n = 16
        legs = [self._leg(1, math.cos(2 * math.pi * k / n), math.sin(2 * math.pi * k / n)) for k in range(n)]
        self._avg_time = sum(t for _, t in legs) / n
        self._avg_drag_energy = sum(d * t for d, t in legs) / n

    # ------------------------------------------------------------ building blocks
    def _leg(self, z: int, ux: float, uy: float) -> tuple[float, float]:
        """(drag power, seconds) for one planned cell along the unit vector (ux, uy) on layer z."""
        key = (z, ux, uy)
        r = self._cell.get(key)
        if r is None:
            h = max(z, 1) * self.layer_m
            wx, wy = self.wind.mean_at(h)
            g = self.wind.gust_at(h)
            vx, vy = ux * self.cruise, uy * self.cruise
            a2 = (vx - wx) ** 2 + (vy - wy) ** 2
            lim = self.vair - g
            if math.sqrt(a2) <= lim:
                t, air2 = self.tick, a2 + g * g
            else:
                # airspeed-limited: the fastest ground speed s along u with |s u - w| = lim
                uw = ux * wx + uy * wy
                disc = lim * lim - (wx * wx + wy * wy) + uw * uw
                s = uw + math.sqrt(disc) if disc > 0 else 0.0
                t, air2 = self.cell_m / max(s, 1.0), self.vair * self.vair
            r = (self.pm.cd * air2 ** 1.5, t)
            self._cell[key] = r
        return r

    def _still_drag(self, z: int, vz: float = 0.0) -> float:
        """Drag power hovering / climbing / descending in the forecast wind on layer z."""
        key = (z, vz)
        p = self._still.get(key)
        if p is None:
            h = max(z, 1) * self.layer_m
            wx, wy = self.wind.mean_at(h)
            g = self.wind.gust_at(h)
            p = self.pm.cd * (wx * wx + wy * wy + g * g + vz * vz) ** 1.5
            self._still[key] = p
        return p

    def _hover_p(self, payload: float) -> float:
        return self.pm.ph * self.pm.hover_factor(payload)

    def _climb_power(self, payload: float, z: int = 1) -> float:
        return self._hover_p(payload) + self._still_drag(z, self.climb_rate) \
            + self.pm.kc * (self.pm.m0 + payload) * G * self.climb_rate

    def _descent_power(self, payload: float, z: int = 1) -> float:
        h = self._hover_p(payload)
        p = h + self._still_drag(z, self.descent_rate) - self.pm.kc * (self.pm.m0 + payload) * G * self.descent_rate
        return max(p, self.pm.floor * h)

    # ------------------------------------------------------ EnergyModel interface
    def move(self, payload: float = 0.0) -> float:
        """One cell, averaged over headings."""
        return (self._hover_p(payload) * self._avg_time + self._avg_drag_energy) * self.margin

    def hover(self, payload: float = 0.0) -> float:
        return (self._hover_p(payload) + self._still_drag(1)) * self.tick * self.margin

    def takeoff(self, payload: float = 0.0) -> float:
        return self._climb_power(payload) * self.t_takeoff * self.margin

    def climb(self, payload: float = 0.0) -> float:
        return self._climb_power(payload) * self.t_climb * self.margin

    def descend(self, payload: float = 0.0) -> float:
        return self._descent_power(payload) * self.t_descend * self.margin

    def land(self, payload: float = 0.0) -> float:
        """The vertical descent from layer 1 to the pad."""
        return self.descend(payload)

    def step(self, payload: float = 0.0) -> float:
        """Bound on one tick of flight for estimates (see :func:`dronefleet.energy.step_cost_bound`)."""
        m = self.move(payload)
        if self.cfg.n_layers == 1:
            return m
        c, d = self.climb(payload), self.descend(payload)
        return max(m, (c + d) / 2, d)

    def transition(self, prev_air: bool, prev_cell, now_air: bool, now_cell, payload: float) -> float:
        """Energy of one planned step, priced with its real direction against the forecast wind."""
        if not now_air:
            return 0.0
        if not prev_air:
            return self.takeoff(payload)
        z = now_cell[2]
        if prev_cell == now_cell:
            return (self._hover_p(payload) + self._still_drag(z)) * self.tick * self.margin
        if prev_cell[0] == now_cell[0] and prev_cell[1] == now_cell[1]:
            if now_cell[2] > prev_cell[2]:
                return self._climb_power(payload, z) * self.t_climb * self.margin
            return self._descent_power(payload, z) * self.t_descend * self.margin
        drag, t = self._leg(z, float(now_cell[0] - prev_cell[0]), float(now_cell[1] - prev_cell[1]))
        return (self._hover_p(payload) + drag) * t * self.margin

    def estimate(self, cells: float, payload: float = 0.0, takeoff: bool = True,
                 hover_ticks: int = 0, slack: float = 1.0) -> float:
        """Fast estimate (bids, reaching a station): heading-averaged price plus one landing."""
        return super().estimate(cells, payload, takeoff, hover_ticks, slack) + self.land(payload)
