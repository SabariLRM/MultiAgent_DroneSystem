# Multi-Agent Drone Fleet Coordination for Package Delivery

A fleet of autonomous delivery drones shares one city's airspace. The drones
**bid for parcels** in an auction, **plan collision-free routes in space and
time** across **stacked altitude layers**, recover when **wind** knocks them
off course, and **swap batteries** at stations before they run dry. Every
interaction between them is a message, and the whole system is plain Python
(standard library only), so it runs anywhere.

Two flight models share the same agents. The **grid** model moves drones one
100 m cell or 30 m layer per 10-second tick. **Continuous flight**
(`--motion continuous`) flies the same plans in metres and seconds: speed and
acceleration limits, a wind field, a power model, and **3-D ORCA** keeping
drones apart. In continuous runs drones **cruise at 60 m and climb over
buildings**, and customers **live in houses and buildings, with parcels
lowered onto roofs at several heights**. Every run can be replayed in an
interactive 2D map or a 3D city.

![Collision avoidance results](results/figures/coordination.svg)

## At a glance

| | |
|---|---|
| **Agents** | `DroneAgent` ×N (hybrid deliberative/reactive), `DispatcherAgent` (Contract Net auctioneer), `SwapStationAgent` ×3 (battery-pack managers), an air-traffic-control broadcaster (no-fly-zone notices) |
| **Airspace** | "2.5-D": flight layers 1..`n_layers` (default 3, at 30/60/90 m) above the ground; buildings have heights and block only the layers at or below them; vertical take-off and landing at pads; optional heading rule (east/west on odd layers, north/south on even) |
| **Route planning** | 3-D space-time A* over a shared reservation table (Cooperative A*), exact 3-D BFS heuristic, vertex and head-on checks (vertical ones too), ground-holding, no-fly-zone time windows and altitude ranges, optional cruise altitude |
| **Collision avoidance** | Reservations (deliberative) + right-of-way safety layer (reactive) + yield requests and priority escalation (negotiation) + plan repair. In continuous flight: the same reservations (strategic) + 3-D ORCA with a 40 m × 15 m separation bubble and vertiport rules (tactical) |
| **Task allocation** | Contract Net Protocol with energy-aware bids, including "swap first" and "after my swap" bids; baselines: nearest-idle and round-robin |
| **Battery** | Every plan is checked against battery ≥ plan + reach a station + reserve; station choice by travel time plus believed queue; emergency diversion; physical pack exchange and charging |
| **Deliveries** | Parcels lowered on a winch: onto open ground (grid default) or, with customers in buildings, into a garden from 30 m and onto 30 m or 60 m roofs from 60 or 90 m |
| **Explainability** | Replays describe every drone's action in plain sentences, and every message between the agents is logged in plain English |

## Results at a glance

10 seeds per configuration, same cities and order streams across strategies.
Full tables: [results/experiments.md](results/experiments.md); discussion:
[docs/REPORT.md](docs/REPORT.md).

| Question | Finding |
|---|---|
| Collision avoidance ([§6.1](docs/REPORT.md#61-collision-avoidance)) | Cooperative reservations: 0 collisions and 0 drones lost in every run. Uncoordinated flight: 63–268 conflicts per run |
| Task allocation ([§6.2](docs/REPORT.md#62-task-allocation)) | Distance-aware allocation beats round robin by about 32–43 % in delivery time; the Contract Net market matches the centralised nearest-idle heuristic without a central planner |
| Battery ([§6.3](docs/REPORT.md#63-battery-management)) | The predictive rule never let a drone run dry in flight; a fixed 30 % threshold lost 5–9 drones per run |
| Scale and infrastructure ([§6.4](docs/REPORT.md#64-scalability-and-infrastructure)) | Throughput scales linearly up to 16 drones; beyond that spare battery packs, not swap bays or planning, are the bottleneck |
| Robustness ([§6.5](docs/REPORT.md#65-robustness-to-execution-noise)) | With 30 % of moves disturbed by gusts: still 0 collisions and 0 depletions |
| Altitude layers ([§6.6](docs/REPORT.md#66-altitude-layers)) | 3 or 5 layers stay collision-free at 24 and 32 drones; with a free choice drones fly above layer 1 only 1–3 % of the time; the heading rule separates crossing traffic but costs 20–30 % in delivery time |
| Continuous flight ([§6.7](docs/REPORT.md#67-continuous-flight)) | Reservations + ORCA: 0 collisions in 60 runs (12 and 24 drones, calm to 8 m/s wind), at most 0.8 losses of separation per run, closest approach 21.7 m. Reservations alone lost separation 27–149 times per run (mostly at pads); ORCA alone collided in 6 of 60 runs |
| Wind ([§6.7](docs/REPORT.md#67-continuous-flight)) | Every accepted order is delivered up to 8 m/s mean wind; at 10 m/s the energy-safe fleet delivers only 20.6 % of orders but loses no drone |
| Cruise and rooftops ([§6.7](docs/REPORT.md#67-continuous-flight)) | Cruising at 60 m: 87 % of flight time at 60 m or higher, buildings crossed at 90 m 82 % of the time, for 30 % longer deliveries and 12 % more energy. Customers in buildings: 67 % of parcels go onto a roof, at almost no extra cost |

A grid run takes about 0.2–1 s, a continuous run 0.6–4 s (32 drones).

## Quick start

Requires Python ≥ 3.10. Nothing to install.

```bash
python3 run_simulation.py
```

```bash
python3 run_simulation.py --motion continuous --view both
```

The first command runs the grid model and writes `results/replay.html`; the
second runs continuous flight and writes `results/replay.html` and
`results/replay_3d.html`. Open them in a browser. Each run also prints its
metrics and writes the agents' messages to `results/replay_messages.txt`.
The 2D replay is self-contained and **works offline**; the 3D replay loads
three.js (a pinned r147 build) from cdn.jsdelivr.net, so it **needs an
internet connection**.

## How a delivery works

1. **Orders arrive** at random (a Poisson stream), each stocked at a hub, with
   a weight and a deadline; some are express.
2. **The dispatcher holds an auction** (Contract Net). Every drone works out
   privately what the order would cost it in energy and when it could deliver,
   including "after a battery swap", and bids or refuses. The best bid wins.
3. **The winner plans a route in space and time** with A* over the shared
   reservation table, books its cells tick by tick, and checks that its
   battery covers the route, the way to a swap station afterwards and a
   reserve.
4. **It flies**: cell by cell in the grid model, or continuously with ORCA
   steering around other drones. Wind gusts and avoidance push it off plan;
   it repairs or replans.
5. **It lowers the parcel on a winch** to the customer, then flies to a hub
   or a **swap station**, where a robot exchanges its pack for a charged one.

## Two flight models

| | Grid (default) | Continuous (`--motion continuous`) |
|---|---|---|
| Space and time | 100 m cells, 30 m layers, 10 s ticks | metres and seconds; physics every 0.5 s, agents still decide every 10 s |
| Motion | one cell or one layer per tick | a path follower flies the planned 4-D waypoints with smoothed corners: 15 m/s, climb 3 m/s, descent 2 m/s, acceleration 4 m/s² |
| Wind | a 3 % chance per move of being held back a tick | a smooth seeded wind field: `--wind calm`, `moderate` (default, 5 m/s), `strong` (8 m/s) or `severe` (10 m/s) |
| Separation | reservations + right-of-way safety layer | the same reservations + 3-D ORCA with a 40 m × 15 m bubble, four touchdown spots per pad |
| Energy | units per move, climb and descent | a power model (hover, drag, climb) calibrated so a 100 m cell at cruise costs the same 1.0 unit |
| Command-line defaults | cruise altitude off, customers on open ground | cruise at 60 m (higher over buildings), customers in houses and buildings |

Design of the continuous mode: [docs/continuous_design.md](docs/continuous_design.md).

**Cruise altitude.** With `--cruise-layer 2` (the continuous default) drones
fly 60 m above whatever is below them: at 60 m over the streets, climbing to
90 m to cross a building and coming back down after it. The planner also
prefers the straight line to the destination, so drones cross the city over
the buildings in their way instead of following the streets; only towers
that reach 90 m are flown around. `--cruise-layer 3` cruises at 90 m
everywhere, `--cruise-layer 0` turns it off.

**Customers in buildings.** With `--customer-buildings` (the continuous
default) each customer is a house, a building with a roof at 30 m, or one
with a roof at 60 m (about a third each). The drone hovers 30 m above the
drop point and winches the parcel down: into the garden from 30 m, onto a
30 m roof from 60 m, onto a 60 m roof from 90 m. `--no-customer-buildings`
turns it off.

Both options are off in `SimConfig` itself, so the experiments and the grid
model are unchanged unless a configuration asks for them.

## Replay viewers

Both viewers share one core: a timeline with play, pause, step and speed; a
fleet list that says what every drone is doing; a follow card for the
selected drone (route, destination, arrival time, battery, height graph and
recent decisions, including why it paused); an event log in plain sentences;
the agents' messages; the swap stations' queues and packs; and a run summary.
Each has a light and a dark theme.

**2D map** (`--view 2d`, the default; works offline): drones labelled with
what they are doing ("→ Hub 1 for #38", "#38 → customer · 60 m"), booked
routes, waiting orders, buildings shaded by height, no-fly zones, parcels
drawn as a box under the drone, and in continuous replays 30-second trails,
an amber ring around any drone ORCA is steering, a red link for every loss
of separation, and the wind.

**3D city** (`--view 3d`): the run as a city at true scale, with towers with
windowed facades, roads, parks, trees, hub and station pads with their four
touchdown spots, the customers' houses and buildings with a landing target
on each roof, a sky and a sun that casts shadows (daytime in the light theme,
dusk with lit windows in the dark theme). Drones are quadcopters with spinning
rotors and navigation lights that lean into their speed; a parcel hangs under
the drone and is lowered on a winch. Continuous replays add trails, velocity
arrows, altitude lines, optional separation bubbles, a wind compass and
windsocks, and for the followed drone a translucent "curtain" from its path
down to the street that shows every climb and descent.

Camera: drag to orbit, right-drag to pan, scroll to zoom down to street
level. Click a drone to follow it with the **Chase camera**, a third-person
camera as in a game: it stays with the drone and turns with it, and you can
drag to look around the drone and scroll to move in or out without leaving
the chase. Press **Chase camera** again to get back behind the drone, or
**Drone view** to ride along just behind it. **Overview**, **Top view** or a
click on empty ground leave the chase.

### Ready-made replays

All in `results/`; open any of them directly.

| File | What it shows |
|---|---|
| `replay.html`, `replay_3d.html` | the default grid run: 12 drones, 3 layers |
| `replay_dense_cooperative.html` (`_3d`) | 24 drones with cooperative planning: 0 collisions |
| `replay_dense_no_coordination.html` (`_3d`) | the same 24 drones without coordination: 222 collisions |
| `replay_naive_battery.html` (`_3d`) | a fixed 30 % battery rule: 4 drones run dry |
| `replay_32_drones_5_layers_heading.html` (`_3d`) | 32 drones, 5 layers, heading rule |
| `replay_continuous.html` (`_3d`) | the default continuous run: 60 m cruise, deliveries onto roofs |
| `replay_continuous_32_drones.html` (`_3d`) | continuous flight with 32 drones |
| `replay_continuous_reservations_only_3d.html` | 24 drones with reservations but no ORCA: red links show the losses of separation, mostly at the pads |
| `replay_messages.txt`, `replay_continuous_messages.txt` | every message of the two default runs, in plain English |

## Agent messages

Every message the agents exchange is written in plain English next to the
replay (`results/replay_messages.txt` by default): who sent what to whom, and
when. For example:

```
 tick  from                 to                   act            message
    2  Dispatcher           all drones           CALL FOR BIDS  Auction round 1: who can deliver this order? #1 (EXPRESS) 1.1 kg Hub 1 -> (20, 15) due t=72
    2  Drone 3              Dispatcher           BID            My bids in round 1: order #1 for cost 33.0, delivered by t=33
  103  Station 1            Drone 10             AGREE          Reserved. Expected wait when you arrive: 31 ticks.
```

The file starts with a count of every kind of message. Routine status reports
(each drone's and each station's, every tick; about 85 % of all messages) are
counted but only listed with `--messages-all`. `--messages FILE` writes the
log elsewhere and `--messages ''` skips it. The replays show the same
sentences in a **Messages between the agents** panel that follows the
timeline (only the followed drone's when you follow one).

## Command-line options (`run_simulation.py`)

| Option | Default | Meaning |
|---|---|---|
| `--seed N` | 7 | random seed: city, orders and wind |
| `--drones N` | 12 | fleet size |
| `--rate R` | 0.16 | orders per tick (Poisson mean) |
| `--ticks N` | 900 | maximum ticks (1 tick = 10 s) |
| `--allocation` | `cnp` | `cnp` (Contract Net) · `nearest` · `round_robin` |
| `--coordination` | `cooperative` | `cooperative` (reservations) · `reactive` (safety layer only) · `none` |
| `--battery` | `predictive` | `predictive` · `naive` (fixed 30 % threshold) |
| `--gust P` | 0.03 | grid model: chance per move of being held back a tick |
| `--layers N` | 3 | flight layers (1 = flat airspace) |
| `--layer-rule` | `free` | `free` · `heading` (east/west on odd layers, north/south on even) |
| `--no-nfz` | | no temporary no-fly zones |
| `--spare-packs N` | 3 | charged spare packs per swap station |
| `--motion` | `grid` | `grid` · `continuous` |
| `--wind` | `moderate` | continuous: `calm` · `moderate` · `strong` · `severe` |
| `--tactical` | `orca` | continuous: `orca` · `none` (reservations only) |
| `--cruise-layer N` | 2 (continuous), 0 (grid) | cruise this many layers above the street or roof below; 0 = off |
| `--customer-buildings` / `--no-customer-buildings` | on (continuous), off (grid) | customers in houses and buildings, parcels onto roofs |
| `--view` | `2d` | `2d` · `3d` · `both` |
| `--out FILE` | `results/replay.html` | replay path (`''` = no replay); `--view both` adds `_3d` |
| `--messages FILE` | next to the replay | plain-English message log (`''` = none) |
| `--messages-all` | | also list the routine status reports |
| `--metrics-json FILE` | | also write the metrics as JSON |
| `--map` / `--events N` | | print the ASCII city map / the first N events |
| `--quiet` | | no progress output |

More scenarios:

```bash
python3 run_simulation.py --drones 24 --rate 0.35                     # a busy city
python3 run_simulation.py --coordination none --drones 24 --rate 0.35   # watch the collisions
python3 run_simulation.py --battery naive --seed 1                     # watch drones run dry
python3 run_simulation.py --layers 5 --layer-rule heading --drones 32 --rate 0.47 --view both
python3 run_simulation.py --motion continuous --wind strong --drones 24 --rate 0.35 --spare-packs 6
python3 run_simulation.py --motion continuous --tactical none           # reservations only, no ORCA
python3 run_simulation.py --motion continuous --coordination none      # ORCA only, no reservations
python3 run_simulation.py --motion continuous --cruise-layer 0 --no-customer-buildings
```

## Experiments

```bash
python3 run_experiments.py                                # all sections: 75 configurations x 10 seeds, about 10 minutes
python3 run_experiments.py --only tactical motion wind    # only the continuous-flight sections, about 7 minutes
python3 run_experiments.py --seeds 3 --jobs 8             # a quick look, in parallel
```

Sections: `coordination`, `allocation`, `battery`, `scalability`,
`infrastructure`, `robustness`, `layers` (grid model) and `tactical`,
`motion`, `wind` (continuous flight). Results go to `results/experiments.md`
and `results/experiments.json`, charts to `results/figures/`. With `--only`
the named sections are replaced and the others kept.

## Tests

```bash
python3 -m unittest discover -s tests -t .
```

135 unit and system tests, about 12 s: planner, reservations, traffic,
agents, altitude layers, replays, continuous flight (controller limits, ORCA
scenarios, power model, wind, smoothing), the message log, cruise altitude,
customers in buildings, and whole runs that must be collision-free, lose no
drone and deliver every order. `tests/test_regression.py` checks that a
one-layer run still reproduces the original flat-airspace metrics exactly.

## Project layout

```
dronefleet/
  config.py              every tunable parameter (SimConfig dataclass), grid and continuous
  world.py               layered city: building heights, pads, customers' drop cells, no-fly zones, 3-D BFS distances, generator
  reservation.py         space-time reservation table (vertex + edge claims, vertical moves included)
  planner.py             3-D space-time A* through chained waypoints (drop -> land), cruise altitude: the strategic layer
  energy.py              battery packs and the grid energy model (move, hover, take-off, climb, descend)
  power.py               continuous power model (hover, drag, climb) and wind-aware pricing
  wind.py                seeded, smooth wind field (mean wind + gust modes + shear)
  orca.py                3-D ORCA half-spaces and linear programs (port of RVO2-3D)
  flight.py              continuous flight: 4-D references, smoothing, path follower, ORCA, vertiports, physics, safety monitor
  messages.py            FIPA-ACL performatives and the message bus
  msglog.py              every agent message as a plain-English sentence
  orders.py              Poisson order stream with a service-area check
  traffic.py             reactive right-of-way resolver + collision detector
  agents/drone.py        the drone agent: beliefs, bidding, mission state machine, planning, repair
  agents/dispatcher.py   Contract Net auctioneer (+ nearest / round-robin baselines)
  agents/station.py      swap-station agent: bays, pack charging, status broadcasts
  simulation.py          the tick loop (environment -> agents -> safety layer -> physics or flight layer)
  metrics.py             performance measures
  replay.py              trace recorder + HTML replay export (2D or 3D)
  viewer_core.js/.css    shared viewer code: wording, side panels, timeline, playback
  viewer_template.html   the 2D viewer (canvas, no external dependencies)
  viewer3d_template.html the 3D viewer (three.js from a CDN)
  charts.py              SVG charts for the report
tests/                   unit and system tests (see above)
run_simulation.py        CLI: one run -> metrics, replay, message log
run_experiments.py       CLI: experiment suite -> results/experiments.md, .json, figures/
docs/REPORT.md           the case-study report: design, algorithms, results, discussion, limitations
docs/continuous_design.md design of the continuous flight mode
results/                 experiment results, charts and ready-made replays
```

## Key parameters (`dronefleet/config.py`)

| Parameter | Default | Meaning |
|---|---|---|
| `n_drones` | 12 | fleet size |
| `n_layers` | 3 | flight layers above the ground (1 = flat airspace) |
| `layer_rule` | `free` | `free` · `heading` |
| `order_rate` | 0.16 | Poisson orders per tick (about 85 orders a run) |
| `allocation` | `cnp` | `cnp` · `nearest` · `round_robin` |
| `coordination` | `cooperative` | `cooperative` · `reactive` · `none` |
| `battery_policy` | `predictive` | `predictive` · `naive` |
| `battery_capacity` | 150 | energy units; one empty cell costs 1.0, +25 % per kg of payload |
| `climb_cost` / `descend_cost` | 1.2 / 0.5 | grid model: energy per layer climbed / descended (take-off 1.5) |
| `gust_prob` | 0.03 | grid model: chance a moving drone is held back a tick |
| `nfz_ceiling` | `None` | generated no-fly zones close layers 1..`nfz_ceiling` (`None` = all) |
| `station_spare_packs` | 3 | charged spare packs per station |
| `motion` | `grid` | `grid` · `continuous` |
| `tactical` | `orca` | continuous: `orca` · `none` |
| `physics_dt` | 0.5 | continuous: physics step (s); agents decide every `tick_s` = 10 s |
| `wind_mean` / `wind_gust` | 5 / 1.5 | continuous: mean wind and RMS gust (m/s) at 30 m |
| `sep_h` / `sep_v` | 40 / 15 | continuous: separation bubble (m) |
| `pad_spots` | 4 | continuous: touchdown spots per hub or station |
| `cruise_layer` | 0 | cruise this many layers above the street or roof below (0 = off) |
| `cruise_penalty` / `cruise_high_penalty` / `cruise_line_penalty` | 2.5 / 0.25 / 1.0 | planning cost per cell and layer below / above the cruise height, and per cell off the straight line |
| `customer_buildings` | `False` | customers in houses and buildings; parcels onto roofs |
| `customer_height_weights` | 0.35 / 0.4 / 0.25 | share of houses / 30 m roofs / 60 m roofs |
| `record_messages` | `False` | keep the plain-English message log (`run_simulation.py` turns it on) |

Scale: one cell is about 100 m, one layer 30 m, one tick 10 s, cruise speed
about 36 km/h (exact in continuous mode, where every physical constant is a
`SimConfig` field).

## Documentation

* [docs/REPORT.md](docs/REPORT.md): the full case study (problem, PEAS,
  architecture, algorithms, experimental method, results, discussion,
  limitations).
* [docs/continuous_design.md](docs/continuous_design.md): how continuous
  flight works and what changed while building it.
* [results/experiments.md](results/experiments.md): every experiment table.
