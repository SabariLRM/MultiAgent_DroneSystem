# Multi-Agent Drone Fleet Coordination for Package Delivery

A fleet of autonomous delivery drones shares one city airspace. They **bid for
parcels** in an auction, **plan collision-free routes in space and time** across
**stacked altitude layers**, get **knocked off course by wind** and recover, and
**swap batteries** at stations before they run dry. Everything is a
message-passing multi-agent system, written in plain Python (standard library
only) so it runs anywhere. Drones either hop between grid cells or, with
`--motion continuous`, **fly in metres and seconds**: the space-time plans
become 4-D waypoints that a path follower flies under speed and acceleration
limits, a wind field and a power model, while **3-D ORCA** keeps drones apart.
Runs can be replayed in a 2D map or a 3D scene.

![Collision avoidance results](results/figures/coordination.svg)

| | |
|---|---|
| **Agents** | `DroneAgent` ×N (hybrid deliberative/reactive), `DispatcherAgent` (Contract Net auctioneer), `SwapStationAgent` ×3 (battery-pack managers), ATC broadcaster (no-fly-zone notices) |
| **Airspace** | "2.5-D": flight layers z = 1..`n_layers` (default 3) above the ground (z = 0); buildings have heights and block only the layers at or below them; vertical take-off and landing at pads; optional heading rule (east/west on odd layers, north/south on even) |
| **Route planning** | 3-D space-time A* + shared reservation table (Cooperative A*), exact 3-D BFS heuristic, vertex + head-on conflict checks (vertical ones too), ground-holding, no-fly-zone time windows and altitude ranges |
| **Collision avoidance** | Cooperative reservations (deliberative) + right-of-way safety layer (reactive) + yield requests and priority escalation (negotiation) + plan repair after disturbances; in continuous flight the reactive layer is 3-D ORCA with vertiport rules (tactical) under the same reservations (strategic) |
| **Task allocation** | Contract Net Protocol with energy-aware bids, including "swap first" and "after my swap" bids; baselines: nearest-idle and round-robin |
| **Battery swaps** | Every plan checked against battery ≥ plan + reach-a-station + reserve (climbs and descents priced per layer); station choice by travel + believed queue; emergency diversion; physical pack exchange and charging |
| **Headline result** | 0 collisions and 0 drones lost across all cooperative runs; uncoordinated flight had 63–268 conflicts per run and a naive battery rule lost 5–9 drones per run (10 seeds each) |
| **Continuous flight** | Strategic reservations + tactical 3-D ORCA: 0 collisions in 60 runs (12 and 24 drones, calm to 8 m/s wind), 0–0.8 losses of separation per run, closest approach 21.7 m. Reservations alone lost separation 27–149 times per run (mostly at pads); ORCA alone collided in 6 of 60 runs. 0.5–3 s per run ([report §6.7](docs/REPORT.md#67-continuous-flight)) |
| **Altitude layers** | 3 or 5 layers stayed collision-free with 24 and 32 drones. With a free choice, drones fly above layer 1 only 1–3 % of the time (passing lanes); at 32 drones 3 layers cut reactive holds by 13 %. The heading rule separates crossing traffic but costs 20–30 % in delivery time and 14 % in energy ([report §6.6](docs/REPORT.md#66-altitude-layers)) |

## Quick start

Requires Python ≥ 3.10. No packages to install.

```bash
python3 run_simulation.py
```

Then open `results/replay.html` in a browser. It is an interactive replay:
every drone is labelled with what it is doing ("→ Hub 1 for #38", "#38 →
customer", "→ Station 2 battery"), and the fleet list and event log explain each
step in plain sentences. Click any drone to follow it: its route, destination,
arrival time, battery and recent decisions are shown, including why it paused
(wind gust, giving way, waiting for a clear route). In stacked airspace every
label also shows the drone's flight layer ("L2").

For a 3D view of the same run (orbit, zoom, click a drone and the camera chases it):

```bash
python3 run_simulation.py --view 3d        # writes results/replay_3d.html
```

The 3D viewer loads three.js (a pinned r147 build) from cdn.jsdelivr.net, so it
**needs an internet connection**; the 2D viewer is fully self-contained and
**works offline**. `--view both` writes both files.

### Continuous flight

```bash
python3 run_simulation.py --motion continuous --view both   # results/replay.html + replay_3d.html
python3 run_simulation.py --motion continuous --wind strong --drones 24 --rate 0.35
python3 run_simulation.py --motion continuous --tactical none   # strategic reservations only, no ORCA
python3 run_simulation.py --motion continuous --coordination none  # ORCA only, no reservations
```

In continuous mode every drone has a position and velocity in metres and
seconds (physics every 0.5 s, agents still decide every 10 s), obeys speed
(15 m/s), climb (3 m/s), descent (2 m/s) and acceleration (4 m/s²) limits,
is pushed by a smooth seeded wind field (`--wind calm | moderate | strong |
severe`: 0, 5, 8 or 10 m/s mean wind; moderate is the default),
and draws its battery through a power model calibrated so that one 100 m cell
at cruise costs the same 1.0 unit as in grid mode. The replay records the
drones' positions every 2 s and is titled "continuous flight": both viewers
draw a 30-second trail behind each drone (smoothed corners, gusts, avoidance
swerves), an amber ring around any drone that ORCA is steering right now, a
red link for every loss of separation, and the wind; the 2D viewer shows
heights in metres, the 3D viewer adds velocity arrows and optional
separation bubbles. As before, **3D replays need an internet connection** to load three.js;
2D replays work offline. Design: [docs/continuous_design.md](docs/continuous_design.md).
Ready-made continuous replays are in `results/`: `replay_continuous.html`
(default run; `_3d` for 3D), `replay_continuous_32_drones*.html`, and
`replay_continuous_reservations_only_3d.html`, reservations without ORCA,
where the red links show the losses of separation at the pads.

Other scenarios:

```bash
python3 run_simulation.py --drones 24 --rate 0.35          # a busy city
python3 run_simulation.py --coordination none --drones 24 --rate 0.35 --out results/no_coordination.html
python3 run_simulation.py --battery naive --seed 1 --out results/naive_battery.html
python3 run_simulation.py --gust 0.2 --map --events 40     # windy day, print map and first events
python3 run_simulation.py --layers 5 --layer-rule heading --drones 32 --rate 0.47 --view both
python3 run_simulation.py --layers 1                       # the original flat airspace
```

Run the controlled experiments (10 seeds × 70 configurations, about 8 minutes):

```bash
python3 run_experiments.py
python3 run_experiments.py --only tactical motion wind   # just the continuous-flight sections (~5 minutes)
```

Run the test suite (111 unit + integration tests, about 8 s):

```bash
python3 -m unittest discover -s tests -t .
```

## Project layout

```
dronefleet/
  config.py              every tunable parameter (SimConfig dataclass), grid and continuous
  world.py               layered city airspace: building heights, pads, no-fly zones, 3-D BFS distances, city generator
  reservation.py         space-time reservation table (vertex + edge claims, vertical moves included)
  planner.py             3-D space-time A* through chained waypoints (drop -> land): the strategic layer
  energy.py              battery packs and the flight-energy model (move, hover, take-off, climb, descend)
  power.py               continuous power model (hover ~ mass^1.5, drag ~ airspeed^3, climb ~ m g v_z) and wind-aware pricing
  wind.py                seeded, smooth spatio-temporal wind field (mean wind + sinusoidal gust modes, shear)
  orca.py                3-D ORCA half-spaces and linear programs (port of RVO2-3D)
  flight.py              continuous flight: 4-D references + smoothing, path follower, ORCA, vertiports, physics, safety monitor
  messages.py            FIPA-ACL performatives and the message bus
  orders.py              Poisson order stream with a service-area check
  traffic.py             reactive right-of-way resolver + collision detector
  agents/drone.py        the drone agent: beliefs, bidding, mission FSM, planning, repair
  agents/dispatcher.py   Contract Net auctioneer (+ nearest / round-robin baselines)
  agents/station.py      swap-station agent: FIFO bays, pack charging, status broadcasts
  simulation.py          the tick loop (environment -> agents -> safety layer -> physics; continuous: flight layer)
  metrics.py             performance measures
  replay.py              trace recorder + HTML replay export (2D or 3D)
  viewer_core.js/.css    shared viewer code: describe() wording, side panels, timeline, playback
  viewer_template.html   the 2D replay viewer (canvas, no external dependencies)
  viewer3d_template.html the 3D replay viewer (three.js from a CDN)
  charts.py              SVG charts for the report
tests/                   unit tests (planner, reservations, traffic, agents, layers, replay) + system tests
  test_regression.py     one-layer runs must reproduce the recorded flat-airspace metrics (tests/data/)
  test_continuous.py     controller limits, ORCA scenarios, power model, wind, smoothing, continuous system runs
run_simulation.py        CLI: one run -> metrics + replay
run_experiments.py       CLI: experiment suite -> results/experiments.md, .json, figures/
docs/REPORT.md           the case-study report (design, algorithms, results, discussion)
docs/continuous_design.md design of the continuous flight mode (units, controller, ORCA, energy, recording)
```

## Key parameters (`dronefleet/config.py`)

| parameter | default | meaning |
|---|---|---|
| `n_drones` | 12 | fleet size |
| `n_layers` | 3 | flight layers above the ground (1 = the original flat airspace) |
| `layer_rule` | `free` | `free` · `heading` (east/west on odd layers, north/south on even) |
| `order_rate` | 0.16 | Poisson orders per tick (~96 orders in 600 ticks) |
| `allocation` | `cnp` | `cnp` · `nearest` · `round_robin` |
| `coordination` | `cooperative` | `cooperative` · `reactive` · `none` |
| `battery_policy` | `predictive` | `predictive` · `naive` |
| `gust_prob` | 0.03 | chance a moving drone is held back one tick |
| `battery_capacity` | 150 | energy units; one empty cell costs 1.0, +25 %/kg payload |
| `climb_cost` / `descend_cost` | 1.2 / 0.5 | energy per layer climbed / descended (take-off 1.5, touch-down free) |
| `nfz_ceiling` | `None` | generated no-fly zones close layers 1..`nfz_ceiling` (`None` = all layers) |
| `station_spare_packs` | 3 | charged spare packs per station |
| `motion` | `grid` | `grid` · `continuous` (metres and seconds, see above) |
| `tactical` | `orca` | continuous: `orca` · `none` (strategic reservations only) |
| `physics_dt` | 0.5 | continuous: physics sub-step (s); agents still decide every `tick_s` = 10 s |
| `wind_mean` / `wind_gust` | 5 / 1.5 | continuous: mean wind and RMS gust (m/s) at 30 m; presets in `wind.py` |
| `sep_h` / `sep_v` | 40 / 15 | continuous: separation bubble (m) |
| `pad_spots` | 4 | continuous: touchdown spots per hub or station |

Scale: one cell ≈ 100 m, one layer ≈ 30 m, one tick ≈ 10 s, cruise ≈ 36 km/h
(exact in continuous mode, where every physical constant is a `SimConfig` field).

See [docs/REPORT.md](docs/REPORT.md) for the full write-up.
