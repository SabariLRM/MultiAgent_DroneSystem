# Multi-Agent Drone Fleet Coordination for Package Delivery

A fleet of autonomous delivery drones shares one city airspace. They **bid for
parcels** in an auction, **plan collision-free routes in space and time**, get
**knocked off course by wind** and recover, and **swap batteries** at stations
before they run dry. Everything is a message-passing multi-agent system, written
in plain Python (standard library only) so it runs anywhere.

![Collision avoidance results](results/figures/coordination.svg)

| | |
|---|---|
| **Agents** | `DroneAgent` ×N (hybrid deliberative/reactive), `DispatcherAgent` (Contract Net auctioneer), `SwapStationAgent` ×3 (battery-pack managers), ATC broadcaster (no-fly-zone notices) |
| **Route planning** | Space-time A* + shared reservation table (Cooperative A*), exact BFS heuristic, vertex + head-on conflict checks, ground-holding, no-fly-zone time windows |
| **Collision avoidance** | Cooperative reservations (deliberative) + right-of-way safety layer (reactive) + yield requests and priority escalation (negotiation) + plan repair after disturbances |
| **Task allocation** | Contract Net Protocol with energy-aware bids, including "swap first" and "after my swap" bids; baselines: nearest-idle and round-robin |
| **Battery swaps** | Every plan checked against battery ≥ plan + reach-a-station + reserve; station choice by travel + believed queue; emergency diversion; physical pack exchange and charging |
| **Headline result** | 0 collisions and 0 drones lost across all cooperative runs; uncoordinated flight had 63–268 conflicts per run and a naive battery rule lost 5–9 drones per run (10 seeds each) |

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
(wind gust, giving way, waiting for a clear route). Other scenarios:

```bash
python3 run_simulation.py --drones 24 --rate 0.35          # a busy city
python3 run_simulation.py --coordination none --drones 24 --rate 0.35 --out results/no_coordination.html
python3 run_simulation.py --battery naive --seed 1 --out results/naive_battery.html
python3 run_simulation.py --gust 0.2 --map --events 40     # windy day, print map and first events
```

Run the controlled experiments (10 seeds × 31 configurations, about 1 minute):

```bash
python3 run_experiments.py
```

Run the test suite (47 unit + integration tests, about 1 s):

```bash
python3 -m unittest discover -s tests -t .
```

## Project layout

```
dronefleet/
  config.py            every tunable parameter (SimConfig dataclass)
  world.py             city grid, buildings, pads, no-fly zones, BFS distance maps, city generator
  reservation.py       space-time reservation table (vertex + edge claims)
  planner.py           space-time A* through chained waypoints (drop -> land)
  energy.py            battery packs and the flight-energy model
  messages.py          FIPA-ACL performatives and the message bus
  orders.py            Poisson order stream with a service-area check
  traffic.py           reactive right-of-way resolver + collision detector
  agents/drone.py      the drone agent: beliefs, bidding, mission FSM, planning, repair
  agents/dispatcher.py Contract Net auctioneer (+ nearest / round-robin baselines)
  agents/station.py    swap-station agent: FIFO bays, pack charging, status broadcasts
  simulation.py        the tick loop (environment -> agents -> safety layer -> physics)
  metrics.py           performance measures
  replay.py            trace recorder + self-contained HTML replay export
  viewer_template.html the replay viewer (canvas, no external dependencies)
  charts.py            SVG charts for the report
tests/                 unit tests (planner, reservations, traffic, agents) + system tests
run_simulation.py      CLI: one run -> metrics + replay
run_experiments.py     CLI: experiment suite -> results/experiments.md, .json, figures/
docs/REPORT.md         the case-study report (design, algorithms, results, discussion)
```

## Key parameters (`dronefleet/config.py`)

| parameter | default | meaning |
|---|---|---|
| `n_drones` | 12 | fleet size |
| `order_rate` | 0.16 | Poisson orders per tick (~96 orders in 600 ticks) |
| `allocation` | `cnp` | `cnp` · `nearest` · `round_robin` |
| `coordination` | `cooperative` | `cooperative` · `reactive` · `none` |
| `battery_policy` | `predictive` | `predictive` · `naive` |
| `gust_prob` | 0.03 | chance a moving drone is held back one tick |
| `battery_capacity` | 150 | energy units; one empty cell costs 1.0, +25 %/kg payload |
| `station_spare_packs` | 3 | charged spare packs per station |

Scale: one cell ≈ 100 m, one tick ≈ 10 s, cruise ≈ 36 km/h.

See [docs/REPORT.md](docs/REPORT.md) for the full write-up.
