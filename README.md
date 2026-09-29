# Multi-Agent Drone Fleet Coordination for Package Delivery

A fleet of autonomous delivery drones shares one city airspace. They **bid for
parcels** in an auction, **plan collision-free routes in space and time** across
**stacked altitude layers**, get **knocked off course by wind** and recover, and
**swap batteries** at stations before they run dry. Everything is a
message-passing multi-agent system, written in plain Python (standard library
only) so it runs anywhere. Runs can be replayed in a 2D map or a 3D scene.

![Collision avoidance results](results/figures/coordination.svg)

| | |
|---|---|
| **Agents** | `DroneAgent` ×N (hybrid deliberative/reactive), `DispatcherAgent` (Contract Net auctioneer), `SwapStationAgent` ×3 (battery-pack managers), ATC broadcaster (no-fly-zone notices) |
| **Airspace** | "2.5-D": flight layers z = 1..`n_layers` (default 3) above the ground (z = 0); buildings have heights and block only the layers at or below them; vertical take-off and landing at pads; optional heading rule (east/west on odd layers, north/south on even) |
| **Route planning** | 3-D space-time A* + shared reservation table (Cooperative A*), exact 3-D BFS heuristic, vertex + head-on conflict checks (vertical ones too), ground-holding, no-fly-zone time windows and altitude ranges |
| **Collision avoidance** | Cooperative reservations (deliberative) + right-of-way safety layer (reactive) + yield requests and priority escalation (negotiation) + plan repair after disturbances |
| **Task allocation** | Contract Net Protocol with energy-aware bids, including "swap first" and "after my swap" bids; baselines: nearest-idle and round-robin |
| **Battery swaps** | Every plan checked against battery ≥ plan + reach-a-station + reserve (climbs and descents priced per layer); station choice by travel + believed queue; emergency diversion; physical pack exchange and charging |
| **Headline result** | 0 collisions and 0 drones lost across all cooperative runs; uncoordinated flight had 63–268 conflicts per run and a naive battery rule lost 5–9 drones per run (10 seeds each) |
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
**works offline**. `--view both` writes both files. Other scenarios:

```bash
python3 run_simulation.py --drones 24 --rate 0.35          # a busy city
python3 run_simulation.py --coordination none --drones 24 --rate 0.35 --out results/no_coordination.html
python3 run_simulation.py --battery naive --seed 1 --out results/naive_battery.html
python3 run_simulation.py --gust 0.2 --map --events 40     # windy day, print map and first events
python3 run_simulation.py --layers 5 --layer-rule heading --drones 32 --rate 0.47 --view both
python3 run_simulation.py --layers 1                       # the original flat airspace
```

Run the controlled experiments (10 seeds × 43 configurations, about 3 minutes):

```bash
python3 run_experiments.py
```

Run the test suite (78 unit + integration tests, about 4 s):

```bash
python3 -m unittest discover -s tests -t .
```

## Project layout

```
dronefleet/
  config.py              every tunable parameter (SimConfig dataclass)
  world.py               layered city airspace: building heights, pads, no-fly zones, 3-D BFS distances, city generator
  reservation.py         space-time reservation table (vertex + edge claims, vertical moves included)
  planner.py             3-D space-time A* through chained waypoints (drop -> land)
  energy.py              battery packs and the flight-energy model (move, hover, take-off, climb, descend)
  messages.py            FIPA-ACL performatives and the message bus
  orders.py              Poisson order stream with a service-area check
  traffic.py             reactive right-of-way resolver + collision detector
  agents/drone.py        the drone agent: beliefs, bidding, mission FSM, planning, repair
  agents/dispatcher.py   Contract Net auctioneer (+ nearest / round-robin baselines)
  agents/station.py      swap-station agent: FIFO bays, pack charging, status broadcasts
  simulation.py          the tick loop (environment -> agents -> safety layer -> physics)
  metrics.py             performance measures
  replay.py              trace recorder + HTML replay export (2D or 3D)
  viewer_core.js/.css    shared viewer code: describe() wording, side panels, timeline, playback
  viewer_template.html   the 2D replay viewer (canvas, no external dependencies)
  viewer3d_template.html the 3D replay viewer (three.js from a CDN)
  charts.py              SVG charts for the report
tests/                   unit tests (planner, reservations, traffic, agents, layers, replay) + system tests
  test_regression.py     one-layer runs must reproduce the recorded flat-airspace metrics (tests/data/)
run_simulation.py        CLI: one run -> metrics + replay
run_experiments.py       CLI: experiment suite -> results/experiments.md, .json, figures/
docs/REPORT.md           the case-study report (design, algorithms, results, discussion)
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

Scale: one cell ≈ 100 m, one layer ≈ 30 m, one tick ≈ 10 s, cruise ≈ 36 km/h.

See [docs/REPORT.md](docs/REPORT.md) for the full write-up.
