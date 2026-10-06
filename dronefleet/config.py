"""Simulation parameters.

Every tunable number in the system lives here so that experiments can vary a
single field (``dataclasses.replace(cfg, n_drones=16)``) and stay reproducible
through ``seed``.

Units: one grid cell is ~100 m, one tick is ~10 s, so a drone cruises at one
cell per tick (~36 km/h). One altitude layer is ~30 m and a drone climbs or
descends one layer per tick. Battery energy is expressed in abstract "energy
units" where flying one empty cell costs ``move_cost``.

With ``motion = "continuous"`` those scales become exact (``cell_m``,
``layer_m``, ``tick_s``): drones move in metres and seconds under speed,
acceleration and wind, and energy comes from a power model calibrated to the
same units (see ``docs/continuous_design.md``). The grid-only fields
(``gust_prob``, ``move_cost``, ...) are then unused by the physics, and the
continuous-only fields are ignored in grid mode.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict

ALLOCATION_STRATEGIES = ("cnp", "nearest", "round_robin")
COORDINATION_MODES = ("cooperative", "reactive", "none")
BATTERY_POLICIES = ("predictive", "naive")
LAYER_RULES = ("free", "heading")
MOTION_MODES = ("grid", "continuous")
TACTICAL_MODES = ("orca", "none")


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
    n_layers: int = 3               # flight layers z = 1..n_layers above the ground (z = 0)
    layer_rule: str = "free"        # "free" | "heading": east/west on odd layers, north/south on even

    # --- fleet -----------------------------------------------------------
    n_drones: int = 12
    battery_capacity: float = 150.0
    move_cost: float = 1.0          # energy per cell flown (empty)
    hover_cost: float = 0.7         # energy per tick spent hovering in place
    takeoff_cost: float = 1.5       # extra energy for a climb-out (ground -> layer 1)
    climb_cost: float = 1.2         # energy per layer climbed (layer z -> z + 1)
    descend_cost: float = 0.5       # energy per layer descended (touch-down itself is free)
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
        # (announce_t, start_t, end_t, (x0, y0, x1, y1)[, (z0, z1)]) -- filled by
        # world gen when ``auto_nfz`` is True; without (z0, z1) a zone closes every layer
    ])
    auto_nfz: bool = True
    nfz_lead_time: int = 8          # ticks of warning before a zone activates
    nfz_ceiling: int | None = None  # generated zones cover layers 1..nfz_ceiling (None = all layers)

    # --- planner ---------------------------------------------------------
    max_expansions: int = 40000     # A* budget per search and per flight layer
    hold_escalation: int = 4        # failed plans before priority escalation
    reservation_hold: int = 3       # ticks a stuck drone reserves its cell

    # --- continuous flight (motion = "continuous"; see docs/continuous_design.md)
    motion: str = "grid"            # "grid": cell hopping | "continuous": metres, seconds, ORCA
    cell_m: float = 100.0           # size of one grid cell
    layer_m: float = 30.0           # height of flight layer 1 (layer z flies at z * layer_m)
    tick_s: float = 10.0            # one agent decision tick
    physics_dt: float = 0.5         # physics / tactical sub-step
    drone_radius_m: float = 0.6
    max_speed_h: float = 15.0       # horizontal speed limit (the plan assumes cell_m / tick_s = 10 m/s)
    max_climb: float = 3.0
    max_descent: float = 2.0
    max_accel: float = 4.0
    track_gain: float = 0.4         # path follower: position gain (1/s), horizontal
    track_gain_v: float = 0.5       # ... and vertical
    brake_decel: float = 3.0        # deceleration used to stop at pads and customers (m/s^2)
    smoothing: bool = True          # any-angle line-of-sight shortcutting of the planned route
    smooth_tol: float = 0.36        # max distance from the planned cell at every half tick (cells)
    building_margin_m: float = 10.0 # clearance kept from building boxes by smoothed routes
    track_tol_h: float = 40.0       # "on plan" at a tick: within this of the planned cell ...
    track_tol_v: float = 12.0       # ... horizontally and vertically (m)
    land_ticks: int = 0             # extra ticks over the pad reserved for the vertical landing (continuous only)
    takeoff_window_s: float = 3.0   # a take-off must start this soon after its planned time, else it waits
    pad_spots: int = 4              # touchdown spots per hub/station (1-4), each with its own column
    pad_spot_offset_m: float = 25.0 # spots sit at (+-offset, +-offset) from the pad centre (50 m apart)
    tactical: str = "orca"          # "orca" | "none" (strategic reservations only)
    sep_h: float = 40.0             # separation bubble: horizontal ...
    sep_v: float = 15.0             # ... and vertical (m)
    sense_radius_m: float = 300.0   # neighbour sensing radius (spatial-hash bucket size)
    orca_horizon_s: float = 8.0
    orca_margin: float = 1.45       # ORCA radius = orca_margin * sep_h in bubble-scaled space (>= sqrt 2 covers the cylinder)
    orca_max_neighbors: int = 8
    stall_s: float = 30.0           # off plan this long without progress -> hand back to the planner
    wind_mean: float = 5.0          # mean wind at wind_ref_height_m (m/s); presets in wind.py
    wind_dir_deg: float = 30.0      # direction the air moves toward (0 = +x, 90 = +y)
    wind_gust: float = 1.5          # RMS gust speed (m/s)
    wind_ref_height_m: float = 30.0
    wind_shear: float = 0.14        # power-law exponent of the wind profile
    wind_modes: int = 8             # sinusoidal gust modes
    wind_response: float = 0.25     # drag coupling of the drone's velocity to the air (1/s)
    wind_estimator_s: float = 3.0   # time constant of the on-board wind estimate
    max_airspeed: float = 20.0      # horizontal airspeed limit (slows the drone in a headwind)
    base_mass_kg: float = 6.0
    hover_power: float = 0.07       # energy units per second hovering at base mass
    drag_power: float = 3.0e-5      # energy units per second per (m/s)^3 of airspeed
    climb_power: float = 2.83e-4    # energy units per joule of lift work (m * g * v_z)
    min_power_fraction: float = 0.4 # power never drops below this share of hover power
    energy_margin: float = 1.0      # planning prices x this (> 1 adds slack for catch-up and avoidance)
    trace_dt: float = 2.0           # replay: continuous positions sampled every trace_dt seconds

    # --- output ----------------------------------------------------------
    record_trace: bool = True
    record_messages: bool = False   # keep a human-readable log of every agent message (msglog.py)

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
        if self.n_layers < 1:
            raise ValueError("need at least one flight layer")
        if self.layer_rule not in LAYER_RULES:
            raise ValueError(f"layer_rule must be one of {LAYER_RULES}")
        if self.nfz_ceiling is not None and self.nfz_ceiling < 1:
            raise ValueError("nfz_ceiling must be None (all layers) or >= 1")
        if self.motion not in MOTION_MODES:
            raise ValueError(f"motion must be one of {MOTION_MODES}")
        if self.tactical not in TACTICAL_MODES:
            raise ValueError(f"tactical must be one of {TACTICAL_MODES}")
        if self.motion == "continuous":
            steps = self.tick_s / self.physics_dt
            if self.physics_dt <= 0 or abs(steps - round(steps)) > 1e-9:
                raise ValueError("physics_dt must divide tick_s")
            every = self.trace_dt / self.physics_dt
            if abs(every - round(every)) > 1e-9 or round(every) < 1:
                raise ValueError("trace_dt must be a multiple of physics_dt")
            if not 1 <= self.pad_spots <= 4:
                raise ValueError("pad_spots must be 1..4")
            if self.max_speed_h < self.cell_m / self.tick_s:
                raise ValueError("max_speed_h must allow the planned cruise speed cell_m / tick_s")
        return self

    def to_dict(self) -> dict:
        return asdict(self)
