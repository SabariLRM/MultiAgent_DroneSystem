"""Delivery orders and the stochastic demand model (Poisson arrivals)."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .energy import step_cost_bound
from .world import GridWorld, Site


@dataclass
class Order:
    oid: int
    hub: Site
    dest: Site
    weight: float
    express: bool
    created_t: int
    deadline_t: int
    status: str = "pending"          # pending|assigned|picked|delivered|failed
    drone: int | None = None
    assigned_t: int | None = None
    picked_t: int | None = None
    delivered_t: int | None = None
    auctions: int = 0                # how many CFP rounds it took to place

    @property
    def priority(self) -> int:
        return 1 if self.express else 0

    def to_msg(self) -> dict:
        return {
            "oid": self.oid, "hub": self.hub, "dest": self.dest, "weight": self.weight,
            "express": self.express, "deadline_t": self.deadline_t,
        }


def _in_range_energy(cfg, world: GridWorld, hub: Site, dest: Site, kg: float) -> float:
    """Conservative energy of station -> hub -> dest -> station on a fresh pack."""
    slack = cfg.detour_factor
    step = step_cost_bound(cfg)             # per tick of flight, climbs and descents included
    fetch = min(world.dist(s, hub) for s in world.stations) * slack * step + cfg.takeoff_cost
    out = world.dist(hub, dest) * slack * step * (1 + cfg.payload_factor * kg)
    back = min(world.dist(dest, s) for s in world.stations) * slack * step
    extra = (cfg.takeoff_cost + cfg.drop_ticks * cfg.hover_cost) * (1 + cfg.payload_factor * kg)
    return fetch + out + back + extra


def _poisson(rng: random.Random, lam: float) -> int:
    # Knuth's method; lam is small (< 1) in practice
    L, k, p = math.exp(-lam), 0, 1.0
    while True:
        p *= rng.random()
        if p <= L:
            return k
        k += 1


class OrderGenerator:
    def __init__(self, cfg, world: GridWorld, flight_energy=None):
        """``flight_energy`` (continuous flight only) is the energy model the drones plan
        with; the service area then also shrinks with the forecast wind."""
        self.cfg = cfg
        self.world = world
        self.flight_energy = flight_energy
        self.rng = random.Random(cfg.seed * 104729 + 3)
        self._next = 1
        self.generated: list[Order] = []

    def _in_range(self, hub: Site, dest: Site, kg: float) -> bool:
        budget = self.cfg.battery_capacity * (0.95 - self.cfg.reserve_fraction)
        if _in_range_energy(self.cfg, self.world, hub, dest, kg) > budget:
            return False
        return self.flight_energy is None or self._in_wind_range(hub, dest, kg, budget)

    def _in_wind_range(self, hub: Site, dest: Site, kg: float, budget: float) -> bool:
        """Could a drone on a fresh pack at the station nearest the hub bid for it?
        (The drones' own "swap first" estimate, with the forecast wind.)"""
        e, w, slack = self.flight_energy, self.world, self.cfg.detour_factor
        fetch = e.estimate(min(w.dist(s, hub) for s in w.stations), 0.0, True, slack=slack)
        out = e.estimate(w.dist(hub, dest), kg, True, self.cfg.drop_ticks, slack)
        back = e.estimate(min(w.dist(dest, s) for s in w.stations), 0.0, False, slack=slack)
        return fetch + out + back <= budget

    def tick(self, t: int) -> list[Order]:
        if t > self.cfg.order_until:
            return []
        out = []
        for _ in range(_poisson(self.rng, self.cfg.order_rate)):
            express = self.rng.random() < self.cfg.express_fraction
            weight = round(self.rng.uniform(0.3, min(2.5, self.cfg.max_payload_kg)), 2)
            # customers outside the service area (beyond one pack's range from
            # their closest warehouse) cannot order by drone
            for _ in range(50):
                dest = self.rng.choice(self.world.customers)
                hubs = sorted(self.world.hubs, key=lambda h: (self.world.dist(h, dest), h))
                if self._in_range(hubs[0], dest, weight):
                    break
            hub = hubs[0]
            # usually fulfilled by the closest warehouse, sometimes the item is
            # only stocked at another one
            if len(hubs) > 1 and self.rng.random() < self.cfg.remote_hub_fraction:
                alt = self.rng.choice(hubs[1:])
                if self._in_range(alt, dest, weight):
                    hub = alt
            budget = self.cfg.express_deadline if express else self.cfg.standard_deadline
            o = Order(self._next, hub, dest, weight, express, t, t + budget)
            self._next += 1
            out.append(o)
        self.generated.extend(out)
        return out
