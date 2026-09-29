"""Simulation parameters.

Every tunable number in the system lives here so that experiments can vary a
single field (``dataclasses.replace(cfg, n_drones=16)``) and stay reproducible
through ``seed``.

Units: one grid cell is ~100 m, one tick is ~10 s, so a drone cruises at one
cell per tick (~36 km/h). Battery energy is expressed in abstract "energy
units" where flying one empty cell costs ``move_cost``.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict

ALLOCATION_STRATEGIES = ("cnp", "nearest", "round_robin")
COORDINATION_MODES = ("cooperative", "reactive", "none")
BATTERY_POLICIES = ("predictive", "naive")


@dataclass
class SimConfig:
    seed: int = 7

    # --- world -----------------------------------------------------------
    width: int = 32
    height: int = 24
    building_density: float = 0.13
    n_hubs: int = 2
    n_stations: int = 3
    n_customers: int = 40

    # --- fleet -----------------------------------------------------------
    n_drones: int = 12
    battery_capacity: float = 150.0
    move_cost: float = 1.0          # energy per cell flown (empty)
    hover_cost: float = 0.7         # energy per tick spent hovering in place
    takeoff_cost: float = 1.5       # extra energy for a climb-out
    payload_factor: float = 0.25    # +25 % flight energy per kg carried
    max_payload_kg: float = 3.0
    loading_ticks: int = 2          # ground time at a hub to load a parcel
    drop_ticks: int = 2             # hover time at the customer (winch down)
    reserve_fraction: float = 0.12  # energy that must remain after any plan
    critical_fraction: float = 0.05 # below this (after remaining plan) -> divert
    swap_threshold: float = 0.40    # idle drones swap proactively below this
    naive_threshold: float = 0.30   # the naive policy's only rule
    detour_factor: float = 1.25     # bid-time slack for detours around traffic
    reposition_after: int = 6       # idle ticks at a station before flying to a hub

    # --- swap stations ---------------------------------------------------
    station_bays: int = 1           # simultaneous swaps per station
    station_spare_packs: int = 3    # charged spare packs at start
    swap_ticks: int = 3
    charge_rate: float = 2.0        # energy units restored per tick per pack

    # --- demand ----------------------------------------------------------
    order_rate: float = 0.16        # Poisson mean orders per tick
    order_until: int = 500          # no new orders after this tick
    express_fraction: float = 0.2
    remote_hub_fraction: float = 0.3  # orders stocked only at a farther hub
    standard_deadline: int = 140    # ticks from order creation
    express_deadline: int = 70
    max_ticks: int = 900

    # --- strategies ------------------------------------------------------
    allocation: str = "cnp"
    coordination: str = "cooperative"
    battery_policy: str = "predictive"
    auction_batch: int = 12         # max orders announced per CFP round
    cfp_backoff: int = 3            # ticks to wait after a round with no awards
    order_expiry: int = 250         # ticks past the deadline before an order is cancelled

    # --- disturbances ----------------------------------------------------
    gust_prob: float = 0.03         # chance a moving drone is held back a tick
    nfz_events: list = field(default_factory=lambda: [
        # (announce_t, start_t, end_t, (x0, y0, x1, y1)) -- filled by world gen
        # when ``auto_nfz`` is True
    ])
    auto_nfz: bool = True
    nfz_lead_time: int = 8          # ticks of warning before a zone activates

    # --- planner ---------------------------------------------------------
    max_expansions: int = 40000
    hold_escalation: int = 4        # failed plans before priority escalation
    reservation_hold: int = 3       # ticks a stuck drone reserves its cell

    # --- output ----------------------------------------------------------
    record_trace: bool = True

    def validate(self) -> "SimConfig":
        if self.allocation not in ALLOCATION_STRATEGIES:
            raise ValueError(f"allocation must be one of {ALLOCATION_STRATEGIES}")
        if self.coordination not in COORDINATION_MODES:
            raise ValueError(f"coordination must be one of {COORDINATION_MODES}")
        if self.battery_policy not in BATTERY_POLICIES:
            raise ValueError(f"battery_policy must be one of {BATTERY_POLICIES}")
        if self.n_drones < 1:
            raise ValueError("need at least one drone")
        if not 0 <= self.gust_prob < 1:
            raise ValueError("gust_prob must be in [0, 1)")
        return self

    def to_dict(self) -> dict:
        return asdict(self)
