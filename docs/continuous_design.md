# Continuous 3-D flight: design plan

`SimConfig.motion = "continuous"` replaces grid-cell hopping with real motion.
`motion = "grid"` (the default) is untouched: every existing test, the recorded
flat-airspace baseline and the committed experiment tables stay valid.

## 1. Hierarchy

Real drone traffic management separates *strategic* deconfliction (flight
plans agreed before or during flight, e.g. a UTM service) from *tactical*
avoidance (detect-and-avoid on board). The simulator mirrors that:

| layer | runs every | what it does | code |
|---|---|---|---|
| Agents (unchanged) | 10 s tick | auctions, battery decisions, swap stations, messages | `agents/` |
| Strategic | 10 s tick | space-time A* + reservation table produce 4-D waypoints: cell centres with target times | `planner.py`, `reservation.py` |
| Tactical | 0.5 s sub-step | path following of those waypoints, 3-D ORCA against nearby drones, vertiport rules | `flight.py`, `orca.py` |
| Physics | 0.5 s sub-step | point-mass dynamics, wind field, power model, separation monitor | `flight.py`, `wind.py`, `power.py` |

The agents never see metres. At every tick boundary the tactical layer reports
a discrete cell back to the drone agent (§5), and the agent's existing logic
(on plan / +1-tick repair / re-plan / hold / yield / escalation) does the rest.

## 2. Units and geometry

* Metres and seconds. A cell is `cell_m = 100` m, so cell `(x, y)` has its
  centre at `((x + 0.5) * 100, (y + 0.5) * 100)`. Flight layer `z` is at
  `z * layer_m` = 30·z m (30/60/90 m with the default 3 layers).
* A building of height `h` layers has its roof at `(h + 0.5) * layer_m`, i.e.
  15 m below the lowest layer that overflies it.
* Cruise speed = `cell_m / tick_s` = 10 m/s, the speed the strategic plan
  assumes. Limits: horizontal 15 m/s, climb 3 m/s, descent 2 m/s,
  acceleration 4 m/s². The 5 m/s horizontal headroom lets a drone catch up
  after wind or an avoidance manoeuvre.
* Drone = point mass with radius 0.6 m. Collision: 3-D distance < 1.2 m.
  Separation bubble: horizontal 40 m and vertical 15 m (a cylinder); a *loss
  of separation* (LoS) is both violated at once.

## 3. Time stepping

The agent decision tick stays 10 s. Between two ticks the physics runs 20
sub-steps of `physics_dt = 0.5` s. Order within a tick:

1. agents deliberate and (re)plan exactly as in grid mode;
2. 20 × { spatial hash → separation monitor → preferred velocity from the
   path follower → ORCA → acceleration-limited dynamics + wind → energy };
3. the tactical layer maps each drone back to a cell, charges the battery and
   reports it; agents observe (`after_move`) as before.

## 4. Strategic → tactical hand-off

* **4-D waypoints.** A committed `Plan` becomes a time-parameterised reference:
  cell centres at `t * tick_s` seconds, linear in between.
* **Any-angle smoothing** (option `smoothing`, on by default): greedy
  line-of-sight shortcutting over runs of horizontal/hover steps. A shortcut
  `i → j` is accepted only if (a) its 3-D segment clears every building box
  (roof + margin, `building_margin_m`), (b) the vertical rate stays within the
  climb/descent limits, and (c) at every half tick it stays within
  `smooth_tol` cells (0.36 by default) of where the plan put the drone. (c)
  keeps the drone inside the space-time tube it reserved, so strategic
  deconfliction still holds; a 90° corner is rounded from edge midpoint to
  edge midpoint, and alternating staircases become straight diagonals.
* **Vertical manoeuvres stay vertical.** Take-off (ground → 30 m at ≤ 3 m/s,
  ≈ 10.75 s with acceleration), the parcel winch (hover over the customer)
  and landing are never smoothed.
* **Landing takes time.** A 30 m descent at 2 m/s takes ≈ 15.5 s, more than a
  tick. In continuous mode the planner therefore appends `land_ticks = 2`
  hover steps over the pad (tag `land_start` → `land`), which also reserves
  the pad column until touch-down. Grid mode has `land_ticks = 0`, so its
  plans are bit-for-bit the same.
* **Descents between layers** take 15 s instead of 10 s; the drone falls
  about 5 s behind and catches up horizontally (they are rare: drones spend
  1–3 % of their time above layer 1).

## 5. Controller (path following)

Per sub-step, with reference position `r(t)` and velocity `r'(t)`:

    v_des = r'(t) + K_p (r(t) - p)          K_p = 0.4 /s horizontal, 0.5 /s vertical

then a braking cap `|v_h| ≤ sqrt(2 · 3 m/s² · distance to the next planned
stop)` so the drone stops smoothly at a pad or customer, and the speed limits.
The dynamics limit the change of velocity to `max_accel · dt`.

Phases: *ground* (parked), *take-off* (vertical climb; only when the pad
column is clear), *fly* (track the reference), *winch* (hold), *land* (align
over the pad, then descend at 2 m/s; only when the column below is clear),
*hold* (no plan: hover at the centre of the held cell).

**Reporting back to the agent.** At a tick boundary the drone is *on plan* if
it is within `track_tol_h` = 40 m horizontally and `track_tol_v` = 12 m
vertically of its planned cell (or in the pad column during its landing
steps); then its cell is the planned one. Otherwise it is the cell its
position falls in (nearest open cell), which makes the agent repair (+1 tick)
or re-plan, exactly like a wind gust in grid mode.

## 6. Tactical avoidance: 3-D ORCA

Optimal Reciprocal Collision Avoidance (van den Berg, Guy, Lin, Manocha 2011),
ported from RVO2-3D (half-space construction, 3-D linear program with the
`linearProgram4` fallback that minimises the largest violation).

* **Anisotropic bubble.** ORCA needs a sphere, the bubble is a cylinder. The
  vertical axis is scaled by `sep_h / sep_v` (40/15 ≈ 2.67) so the bubble
  becomes round; the ORCA radius is `orca_margin · sep_h` (1.25 · 40 = 50 m)
  in that space. Adjacent flight layers (30 m apart = 80 m scaled) never
  interact; the vertical speed limits become two extra LP half-spaces.
* **Neighbours** are found with a spatial hash (buckets of `sense_radius_m` =
  300 m; a drone looks at 3 × 3 buckets), at most `orca_max_neighbors`
  nearest, time horizon `orca_horizon_s` = 8 s.
* **Responsibility.** Cooperative drones take half of the avoidance each. A
  drone lowering a parcel, climbing out of or descending onto a pad is
  treated as non-cooperative (static or moving vertically): the others take
  full responsibility.
* **Vertiport rules.** A drone takes off only when no airborne drone is inside
  the bubble around the pad column up to layer 1; it starts or continues a
  landing descent only when nobody is below it inside the bubble.
* **Symmetry breaking.** A small deterministic "keep right" rotation of the
  preferred velocity when a neighbour is close, the usual fix for perfectly
  symmetric head-on encounters.
* **Deadlocks.** If a drone has been off plan for `stall_s` = 30 s without
  getting closer to its goal, the tactical layer hands it back to the
  strategic layer through the existing hold → yield → escalation logic
  (`hold_streak` counts stalls, so the 4th consecutive stall escalates).
* **Fallback.** If pure-Python ORCA were too slow, sampled-velocity RVO would
  replace it; the target is < 5 s per default run and < 20 s at 32 drones.

`tactical = "none"` switches ORCA and the vertiport rules off (strategic
reservations only); `coordination = "none"` with `tactical = "orca"` is ORCA
without reservations.

## 7. Wind

A smooth, seeded field (`wind.py`, own random stream):

    w(x, y, z, t) = profile(z) · ( mean + Σ_i a_i d_i sin(k_i · (x, y) − ω_i t + φ_i) )

8 modes with random directions `d_i`, wavelengths 400–2000 m and periods
20–120 s; amplitudes give an RMS gust of `wind_gust`. `profile(z) = (z /
30 m)^0.14` (power-law shear). Presets: calm (0, 0), moderate (5 m/s,
1.5 m/s RMS, the default), strong (10 m/s, 3 m/s RMS).

Coupling: drag pulls the drone's velocity toward the air at rate `D =
wind_response` (0.25 /s); the flight controller compensates with a wind
estimate low-pass filtered over `wind_estimator_s` = 3 s, so steady wind is
cancelled and gusts push the drone until the estimate catches up. The
horizontal airspeed is capped at `max_airspeed` = 20 m/s, so a strong
headwind slows the drone below the planned 10 m/s. This replaces the i.i.d.
"held back a tick" gusts (`gust_prob` is ignored in continuous mode).

## 8. Energy

    P = P_hover · (m / m0)^1.5 + c_d · |v_air|³ + k_c · m · g · v_z      (≥ 0.4 · hover)

with m0 = 6 kg, m = m0 + payload. Calibration (no wind, no payload):

| manoeuvre | continuous | grid |
|---|---:|---:|
| cruise one 100 m cell at 10 m/s | 0.07 + 3·10⁻⁵ · 10³ = 0.10 /s × 10 s = **1.00** | 1.0 |
| hover one tick | 0.70 | 0.7 |
| climb one layer at 3 m/s (k_c = 2.83·10⁻⁴ /J) | 1.20 | 1.2 |
| descend one layer at 2 m/s (15 s) | ≈ 0.55 | 0.5 |
| payload 1 / 2 kg on hover | ×1.26 / ×1.54 | ×1.25 / ×1.5 |

One energy unit is then ≈ 8.6 kJ (hover ≈ 600 W, pack ≈ 360 Wh).

The agents' energy model becomes `ContinuousEnergyModel`, derived from the
same power model and the known wind forecast (mean wind + RMS gust), so bids,
battery thresholds and the **predictive invariant** stay meaningful:

* `plan_energy` prices every planned step with its real direction against the
  forecast wind (headwind cells cost more, tailwind cells less); landing
  steps are priced as hovering, which over-counts the actual descent;
* fast estimates (bids, "reach a station") use the direction-averaged price
  plus one landing, with the usual detour factor;
* the four enforcement points of §4.4 are unchanged; the battery is charged
  with the energy integrated by the physics, and the per-tick emergency check
  catches any drift between forecast and actual wind.

## 9. What gets recorded

Metrics (continuous mode only, added to the usual ones):

* `collisions` — pairs closer than 2 × radius (events);
* `separation_losses` (events), `separation_loss_s` (pair-seconds);
* `min_separation_m` (smallest 3-D distance between airborne drones);
* `tracking_error_mean_m`, `tracking_error_p95_m` (distance to the reference);
* `orca_interventions` and `orca_per_drone_hour` (episodes in which ORCA
  changed the preferred velocity by more than 0.5 m/s), `flight_hours`;
* `stall_replans`, `pad_wait_s`, `building_intrusions` (must be 0).

Replay trace: the existing per-tick frames (discrete state for the side
panels) plus, per drone, positions every `trace_dt` = 2 s, quantised to 1 m
and delta-encoded in one flat integer array; LoS episodes, ORCA episodes,
collisions and the wind at the city centre as small lists. Viewers
interpolate between samples; replays stay under 5 MB. Grid replays have no
track and play exactly as before.
