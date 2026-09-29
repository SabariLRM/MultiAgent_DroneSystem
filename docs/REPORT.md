# Multi-Agent Drone Fleet Coordination for Package Delivery — Case Study

**Course:** Foundations of AI · **Artefact:** `dronefleet` (Python, standard library only) ·
**Reproduce:** `python3 run_experiments.py` (all numbers below come from that script, 10 seeds per configuration)

---

## 1. Problem

A city runs an on-demand parcel service with a fleet of battery-powered drones.
Orders arrive at random and must be flown from one of two warehouses (hubs) to a
customer before a deadline (140 ticks ≈ 23 min standard, 70 ticks ≈ 12 min
express). Three things make this hard, and they interact:

1. **Allocation.** Which drone should take which parcel, given each drone's
   position, battery and current commitments? Most of that information exists
   only on the drone.
2. **Conflict-free motion.** Up to 32 drones share one airspace, with
   buildings, temporary no-fly zones and wind gusts. Two drones must never occupy
   the same cell, and never pass through each other head-on.
3. **Energy.** A pack lasts roughly two deliveries. Drones must swap batteries at
   a few stations with limited bays and a limited stock of charged packs, and
   they must never run dry in the air.

The goal is a system that is **safe by construction** (no collisions, no
depletion), **efficient** (fast, on-time, energy-frugal), **robust** to
execution noise, and **decentralised** wherever the information is.

## 2. Task environment

### 2.1 PEAS description of a drone agent

| | |
|---|---|
| **Performance** | on-time delivery rate, delivery time (mean, p95), zero collisions, zero in-flight depletion, energy per delivery |
| **Environment** | 32×24-cell city (≈3.2×2.4 km); 13 % buildings; 2 hubs, 3 swap stations, 40 customer sites; temporary no-fly zones; wind; 11+ other drones |
| **Actuators** | move N/S/E/W, hover, take off, land, winch a parcel down, send messages (bid, accept, request swap, yield …) |
| **Sensors** | own position and battery; messages from the dispatcher, stations and air-traffic control; neighbours' broadcast intents (V2V / ADS-B-like) |

### 2.2 Environment properties (Russell & Norvig)

| property | value | why |
|---|---|---|
| Observability | **partial** | a drone knows other drones only through reservations and broadcasts; station queues through (one-tick-stale) status messages; future no-fly zones only once announced |
| Agents | **multi-agent, cooperative** | shared goal, but competition for airspace, parcels and battery packs |
| Determinism | **stochastic** | Poisson order arrivals, wind gusts (a moving drone is held back with probability *p*) |
| Episodic? | **sequential** | a battery decision now constrains every later mission |
| Dynamics | **dynamic** | the world changes while agents deliberate (orders, zones, other drones) |
| State/time | **discrete** | grid cells, 10-second ticks |
| Model | **known** | the energy model and map are known to the agents |

## 3. Architecture

```mermaid
flowchart LR
    ENV[Environment<br/>orders · wind · no-fly zones] -->|new orders| D
    ATC[ATC broadcaster] -->|INFORM nfz| DR
    D[DispatcherAgent<br/>Contract Net manager] -->|CFP / ACCEPT / REJECT| DR
    DR[DroneAgent ×N<br/>deliberative + reactive] -->|PROPOSE / REFUSE / INFORM delivered| D
    DR -->|REQUEST reserve_swap / swap| S[SwapStationAgent ×3]
    S -->|AGREE / INFORM status, swap_done| DR
    DR <-->|claims / yield requests| RT[(Space-time<br/>reservation table)]
    DR -->|intents| SAFE[Reactive right-of-way layer]
    SAFE --> PHY[Physics: motion, energy,<br/>collision detector]
```

All inter-agent interaction is **message passing** with FIPA-ACL performatives
(`CFP, PROPOSE, REFUSE, ACCEPT_PROPOSAL, REJECT_PROPOSAL, REQUEST, AGREE,
INFORM, FAILURE, CANCEL`). No agent reads another agent's internal state. The
one shared structure is the **reservation table**, a blackboard of airspace
claims. It plays the role of a UTM (UAS Traffic Management) service in real
drone operations.

Each tick runs in a fixed order: environment → dispatcher → stations → drones
deliberate (highest right-of-way first) → same-tick yield negotiation → intents
→ wind → reactive resolution → physics → drones observe outcomes. Messages sent
to an agent that has already acted this tick are read next tick, so protocols
have realistic latency: CFP → bid → award takes two ticks.

### 3.1 The agents

**Drone agent: a hybrid (layered) architecture.**

* *Beliefs:* own position and battery pack; station status (queue, charged packs, estimated wait) from broadcasts; known no-fly zones; its committed plan.
* *Desires:* deliver parcels quickly; never breach the energy reserve; never collide.
* *Intentions:* the committed mission and its space-time plan, reserved in the table.

It is implemented as a finite-state mission controller:

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> TO_PICKUP: won auction
    IDLE --> LOADING: won auction at the hub
    IDLE --> TO_STATION: battery low / "swap first" bid won
    TO_PICKUP --> LOADING: land at hub
    LOADING --> TO_CUSTOMER: plan checked vs energy
    TO_CUSTOMER --> RETURNING: parcel lowered (drop_done)
    TO_CUSTOMER --> TO_STATION: drop_done, next pad is a station
    RETURNING --> TO_PICKUP: won auction in flight
    RETURNING --> IDLE: land at hub
    TO_CUSTOMER --> TO_STATION: battery emergency (keeps parcel)
    TO_STATION --> QUEUED: land at station
    QUEUED --> SWAPPING: bay + charged pack free
    SWAPPING --> IDLE: swap_done (resumes any task)
    TO_PICKUP --> DEAD: battery = 0 in flight
    TO_CUSTOMER --> DEAD: battery = 0 in flight
```

**Dispatcher agent: Contract Net manager.** It holds the order book. Each round
it broadcasts a CFP for up to 12 open orders, collects bids, and awards greedily:
express orders first, then by deadline, each to the cheapest bidder not yet
awarded this round. Awarded drones can still decline (`FAILURE`), and the order
is re-queued. After a round with no awards the dispatcher backs off 3 ticks, so
unplaceable orders do not flood the network.

**Swap station agent: resource manager.** It has 1 robotic bay and 3 spare packs.
It serves landed drones first-come-first-served, and a swap needs a free bay
*and* a pack charged to ≥95 %. Depleted packs go on the charger (2 units/tick).
It broadcasts its status every tick and accepts reservations with an ETA, which
feed its wait estimate.

## 4. Algorithms

### 4.1 Route planning: space-time A* with a reservation table

The search state is `(cell, t, airborne)`. Actions per tick: move to one of 4
neighbours, hover, or, while on a pad, wait on the ground or take off.
Cooperative A* (Silver, 2005) plans drones one at a time. Each drone treats
everyone else's claims as obstacles **in time**:

* **Vertex claim** `(cell, t)` — no two drones in one cell at one time.
* **Edge claim** `(a → b, t)` — blocks the opposite move `b → a` at the same tick. This is the head-on "swap" collision a vertex check alone misses.

Three design choices make it both safe and fast:

1. **Every air segment ends with a landing.** A landed drone is outside the
   airspace, so a plan never needs an infinite "stay at goal" reservation, and
   later planners cannot box it in. Ground waiting at a pad is free and always
   safe, so blocked take-offs simply wait on the pad.
2. **Exact heuristic.** h = BFS distance around buildings (cached per goal).
   Other drones and no-fly zones only lengthen paths, so h is admissible and
   consistent. When the customer lies inside a no-fly zone, the planner uses
   h = max(distance, zone_end − t). That keeps A* admissible while stopping it
   from flooding space-time with hover states: the drone waits on the ground and
   takes off just in time.
3. **Chained waypoints.** `drop` (hover for the winch time, all ticks free) →
   `land`. The whole mission leg is planned and reserved atomically.

In the default scenario a search expands about 18 nodes on average and takes
0.06–0.3 ms.

### 4.2 Collision avoidance in depth

| layer | mechanism | handles |
|---|---|---|
| Deliberative | reservation-table planning (4.1) | all *planned* conflicts |
| Negotiation | a stuck drone *holds* its cell and sends `REQUEST yield` to anyone whose claim it overrides; those drones re-plan in the same tick. After 4 failed attempts a drone escalates priority and plans through lower-ranked drones' claims (never through a drone lowering a parcel). The rank order is strict, so this cannot livelock. | local deadlocks |
| Repair | a drone that fell one tick behind (wind) first tries to shift its remaining plan +1 tick; if the shifted claims are free it keeps them, otherwise it re-plans | execution noise, cheaply |
| Reactive | before moving, every drone broadcasts its next cell. Right-of-way: hovering drones keep their cell; airborne traffic beats a take-off; then express-carrier > carrier > empty > lower id; a head-on pair stops. The rule is re-applied until nothing changes (each pass only turns movers into holders, so it terminates, and distinct cells stay distinct). | anything the layers above missed |
| Monitor | an independent detector counts vertex and edge conflicts | measurement only |

### 4.3 Task allocation: the Contract Net Protocol

```mermaid
sequenceDiagram
    participant D as Dispatcher
    participant A as drone9 (idle at hub1)
    participant B as drone4 (returning, 35%)
    D->>A: CFP [order #38 EXPRESS, #39]
    D->>B: CFP [order #38 EXPRESS, #39]
    A->>D: PROPOSE #38 cost 10.1, #39 cost 22.4
    B->>D: PROPOSE #38 cost 31.0 (via a swap at station0 first)
    D->>A: ACCEPT_PROPOSAL #38 (express first, cheapest)
    D->>B: REJECT_PROPOSAL
    A->>D: INFORM picked_up #38 … INFORM delivered #38
```

(The exchange above is illustrative. The award of express order #38 to drone9
at cost 10.1 is taken from the seed-7 event log.) Each drone bids from
**private knowledge**:

* `cost = ETA − now + 3·lateness + 5·energy/capacity`
* The ETA includes the flight to the hub, loading, the leg to the customer and the winch time.
* The energy check covers hub → customer (with payload) → nearest station, times a detour factor of 1.25, plus a 12 % reserve.

A drone that cannot afford the job **still bids**, for a *swap-first* mission
(fly to the best station, swap, then do the job) at the correspondingly later
ETA. A drone already heading to or waiting at a station bids for work that
starts *after* its swap. The auction therefore trades off "a close drone that
must swap" against "a far drone with charge" without the dispatcher knowing
any battery levels.

### 4.4 Battery management: the predictive policy

Invariant: **every committed plan satisfies**

> `battery ≥ energy(plan) + energy(landing pad → nearest station) + reserve`

The middle term matters. A drone must never land somewhere it cannot leave to
recharge. The invariant is enforced at four points:

1. when bidding (estimate)
2. when committing a delivery leg (exact energy of the A* plan; choose between landing at a hub or a station)
3. when repairing a plan
4. every airborne tick (emergency divert to the nearest station if wind has eaten the margin)

Station choice minimises `travel + believed queue wait`, using the stations'
broadcast status and ETA reservations. Idle drones below 40 % swap
proactively. The **naive baseline** only follows "accept jobs above 30 %, swap
below 30 %, divert below 10 %".

## 5. Experimental method

* **Scenario:**
  * City: 32×24 cells, 2 hubs, 3 stations (1 bay, 3 spare packs each).
  * Fleet: 12 drones, packs of 150 units.
  * Demand: 0.16 orders/tick for 500 ticks (≈85 orders), 20 % express.
  * Environment: 3 % gust probability, 2 announced no-fly zones per run.
* **Controlled comparison:** every configuration runs on the **same 10 seeds**
  (same city, same order stream, same gusts). Only the strategy differs.
  Tables give mean ± standard deviation. Raw per-run metrics are in
  `results/experiments.json`.
* **Dense scenario:** 24 drones, 0.35 orders/tick.
* **Runtime:** the whole suite (310 simulations) runs in ≈1 minute on a laptop.

## 6. Results

### 6.1 Collision avoidance

![coordination](../results/figures/coordination.svg)

| configuration | collisions | delivered | avg time | p95 time | on time | drones lost | reactive holds |
|---|---:|---:|---:|---:|---:|---:|---:|
| cooperative (12 drones) | **0** | 100.0% | 34.8 ±6.8 | 70.5 ±13.4 | 99.0% | **0** | 4 |
| reactive only (12 drones) | 0 | 99.7% | 38.9 ±10.1 | 83.7 ±26.0 | 97.4% | 0.4 | 206 |
| none (12 drones) | 63.2 ±26.7 | 100.0% | 32.6 ±5.0 | 66.5 ±9.7 | 99.1% | 0 | – |
| cooperative (24 drones) | **0** | 100.0% | 57.9 ±24.6 | 142 ±71 | 86.8% | **0** | 19 |
| reactive only (24 drones) | 0 | 97.1% | 90.6 ±39.7 | 230 ±112 | 73.1% | 2.5 | 970 |
| none (24 drones) | 268 ±86 | 100.0% | 49.9 ±20.8 | 123 ±68 | 90.9% | 0 | – |

* **Without coordination, conflicts grow super-linearly with density.** They
  rise from 63 to 268 per run (4.2×) when the fleet doubles. In "none" mode
  conflicts are counted but drones fly on, so its delivery time is an
  optimistic lower bound: in reality each conflict is a potential crash.
* **The reactive layer alone is safe but inefficient.** With no look-ahead,
  drones meet head-on and gridlock at bottlenecks, such as the corridor to a
  station. At 24 drones delivery time is 56 % worse than cooperative. Some
  drones even run dry while stuck in gridlock (2.5 per run), because the jam
  blocks the emergency diversion too.
* **Cooperative reservations do both.** They give 0 collisions at a cost of only
  ≈7 % in delivery time versus the unsafe lower bound (34.8 vs 32.6 ticks), and
  the reactive layer only has to intervene ≈4 times per run, for gust fallout.

### 6.2 Task allocation

![allocation](../results/figures/allocation.svg)

| configuration | avg time | p95 time | express avg | on time | energy/delivery | msgs (excl. telemetry) |
|---|---:|---:|---:|---:|---:|---:|
| contract net (12) | 34.8 ±6.8 | **70.5** ±13.4 | 32.1 | 99.0% | **51.1** | 1517 |
| nearest idle (12) | 34.5 ±7.5 | 75.7 ±18.3 | 30.5 | 98.9% | 52.1 | 575 (+6.7k telemetry) |
| round robin (12) | 51.3 ±7.8 | 91.5 ±13.0 | 45.8 | 97.1% | 62.8 | 634 |
| contract net (24) | **57.9** ±24.6 | 142 ±71 | 46.1 | 86.8% | **55.4** | 4224 |
| nearest idle (24) | 61.6 ±20.6 | 140 ±48 | 40.5 | 89.4% | 57.2 | 1272 |
| round robin (24) | 102 ±28 | 203 ±52 | 51.1 | 69.4% | 64.0 | 1347 |

* **Distance-aware allocation beats round robin by ~32–43 %** in delivery time.
  Round robin also burns 15–23 % more energy per parcel and needs more swaps.
* **The market matches the centralised heuristic without centralising
  information.** Contract Net is within noise of "nearest idle drone" on
  delivery time. It is slightly better on average (and uses 2–3 % less
  energy), and slightly worse on express orders. The
  difference is architectural. The nearest-idle dispatcher needs a continuous
  stream of telemetry from every drone (~6,700 messages per run) and knows
  nothing about batteries. The Contract Net dispatcher needs no drone state at
  all; each drone prices its own battery, swap plans and position into its bid.
* **Cost of the market:** bids and rejections, about 2.6× the non-telemetry
  traffic of the centralised dispatcher. Honest limitation: awards are final.
  If a better drone frees up one tick later, the order is not re-auctioned (§8).

### 6.3 Battery management

![battery](../results/figures/battery.svg)

| configuration | drones lost | orders lost | delivered | emergencies | swaps | swap wait | avg time |
|---|---:|---:|---:|---:|---:|---:|---:|
| predictive (12) | **0** | **0** | **100%** | 0.2 | 44.4 | 2.7 | 34.8 |
| naive 30 % (12) | 4.8 ±3.1 | 2.2 ±1.6 | 97.4% | 6.3 | 27.8 | 2.7 | 47.2 |
| predictive (24) | **0** | **0** | **100%** | 0.1 | 101 | 28.9 | 57.9 |
| naive 30 % (24) | 8.6 ±3.5 | 4.7 ±3.1 | 97.4% | 11.5 | 60.7 | 20.2 | 44.9 |

* **A fixed threshold is not a safety rule.** A drone at 31 % happily accepts a
  long, heavy mission. On average the naive fleet **loses 40 % of its drones**
  in a run (4.8 of 12), each carrying or reserving a parcel.
* **The predictive invariant held in every run:** zero in-flight depletion.
  The 3 emergency diversions in 20 runs are the fourth enforcement point
  (§4.4) working as designed. In each case a no-fly zone was announced
  *mid-flight* over the customer. The re-plan would have hovered until the
  zone lifted, which would breach the reserve. So the drone diverted to a
  station with the parcel still on board, swapped, and finished the delivery
  from the ground once the zone lifted.
* It swaps more often (44 vs 28), because it refuses to start a mission it
  cannot finish. That is the price of safety, and at 24 drones it exposes the
  next bottleneck (§6.4). Naive's lower 24-drone delivery time is survivor
  bias: its failed orders never enter the average.

### 6.4 Scalability and infrastructure

![scalability](../results/figures/scalability.svg)

| drones | orders | deliveries/100 ticks | avg time | on time | swap wait | ms per A* |
|---:|---:|---:|---:|---:|---:|---:|
| 4 | 24.5 | 4.45 | 35.3 | 99.6% | 0.0 | 0.31 |
| 8 | 55.4 | 9.68 | 34.9 | 98.8% | 0.5 | 0.07 |
| 12 | 84.7 | 15.0 | 34.3 | 99.3% | 2.4 | 0.20 |
| 16 | 110 | 19.0 | 31.8 | 99.2% | 6.0 | 0.22 |
| 24 | 167 | 25.3 | 47.6 | 92.1% | 24.6 | 0.10 |
| 32 | 217 | 28.1 | 73.0 | 80.6% | 59.5 | 0.11 |

Up to 16 drones, throughput scales linearly and service quality is flat. Beyond
that, delivery time rises. Planning is **not** the bottleneck: per-search cost
stays under 0.35 ms, and zero collisions hold at every size. The swap-station
queue is, rising from 2 to 60 ticks of wait. A follow-up experiment varies the
infrastructure at 24 drones:

![infrastructure](../results/figures/infrastructure.svg)

| 24 drones | swap wait | max queue | avg time | p95 time | on time |
|---|---:|---:|---:|---:|---:|
| 1 bay, 3 spare packs | 28.9 | 7.0 | 57.9 | 142 | 86.8% |
| 1 bay, 6 spare packs | **0.9** | 2.5 | **29.5** | **55** | **99.9%** |
| 2 bays, 3 spare packs | 28.8 | 7.3 | 55.9 | 135 | 88.6% |
| 2 bays, 6 spare packs | 1.4 | 3.6 | 30.1 | 56 | 99.8% |

**Spare packs, not swap bays, are the binding constraint.** A swap takes 3
ticks, but recharging a pack takes ~40. Doubling packs halves delivery time and
restores 99.9 % on-time service; a second robot arm does almost nothing. The
simulation turns an agent-level policy question into a concrete infrastructure
recommendation.

### 6.5 Robustness to execution noise

![robustness](../results/figures/robustness.svg)
![robustness service](../results/figures/robustness_service.svg)

| gust p | collisions | deviations | repairs | plans | yields | avg time | on time | energy/delivery |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.00 | 0 | 0 | 0 | 66 | 0 | 32.0 | 99.1% | 49.6 |
| 0.05 | 0 | 186 | 164 | 89 | 5.6 | 34.8 | 99.0% | 51.9 |
| 0.10 | 0 | 388 | 342 | 121 | 10.3 | 39.3 | 97.9% | 54.6 |
| 0.20 | 0 | 914 | 803 | 196 | 21.1 | 49.4 | 94.8% | 61.0 |
| 0.30 | 0 | 1621 | 1434 | 287 | 45.2 | 67.7 | 85.4% | 69.0 |

Even when 30 % of all moves are disturbed (≈1,600 deviations per run), the
fleet has **zero collisions and zero depletions**. The cheap +1-tick repair
absorbs ≈88 % of deviations without a new search. Service degrades gracefully:
time and energy rise because drones hover and wait, not because anything
fails.

## 7. Discussion: what the case study shows about AI techniques

* **Search is the workhorse.** A* with an admissible, domain-exact heuristic,
  lifted into space-time, turns "don't collide" from a runtime reflex into a
  planning constraint. The key modelling move is choosing the state space
  (adding `t` and `airborne`) so that safety becomes a property of every plan.
* **Layering deliberation over reaction beats either alone.** Pure reaction
  deadlocks. Pure deliberation breaks under noise. Together (plan, repair,
  negotiate, react) they are safe *and* efficient: a hybrid agent architecture.
* **Markets are a natural fit for distributed knowledge.** The Contract Net lets
  each drone price its private battery state and even conditional plans ("I
  can do it after a swap"). The allocation quality comes from the bids, not from
  a planner that sees everything.
* **Utility is more than distance.** Bids combine time, lateness and energy.
  The battery invariant is a hard constraint layered on top of utility. The
  naive policy shows what happens when a safety constraint is approximated by
  a heuristic threshold.
* **Multi-agent systems have emergent bottlenecks.** No single agent is "wrong"
  at 32 drones; the shared charging resource saturates. Systematic experiments
  (same seeds, one factor at a time) are how such properties are found.

## 8. Limitations and future work

| limitation | possible extension |
|---|---|
| 2-D grid, one altitude | altitude layers by heading (as in UTM concepts): a 3-D state space multiplies capacity |
| Plans are sequential (Cooperative A*): fast but not optimal, and priority-order dependent | Conflict-Based Search or windowed WHCA* with rolling horizons for larger fleets |
| The reservation table is a shared service (a UTM provider) | fully peer-to-peer claims with gossip and conflict resolution under message loss |
| Perfect, lossless communication | message loss and latency; heartbeat time-outs; re-auction on silence |
| Awards are final; single-parcel missions | task re-allocation / decommitment; multi-drop routing (VRP with energy constraints) |
| Swap stations are passive FIFO servers | stations that schedule charging and reserve packs; demand-aware pre-positioning of idle drones (learning from order history) |
| Wind is i.i.d. per move | spatially correlated wind fields with energy effects |

## 9. How to reproduce

```bash
python3 -m unittest discover -s tests -t .      # 47 tests: planner, reservations, traffic, agents, system
python3 run_experiments.py --seeds 10           # tables -> results/experiments.md, charts -> results/figures/
python3 run_simulation.py                       # one run -> results/replay.html (interactive)
```

The system-level tests encode the headline guarantees on three seeds: no
collisions, no depletion, every order delivered, no no-fly-zone violations,
battery packs conserved, determinism. They also check that the baselines fail
as expected: uncoordinated flight collides, and the reactive layer alone does not.

## References

* R. G. Smith (1980). *The Contract Net Protocol: High-Level Communication and Control in a Distributed Problem Solver.* IEEE Trans. Computers.
* D. Silver (2005). *Cooperative Pathfinding.* AIIDE.
* P. E. Hart, N. J. Nilsson, B. Raphael (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths.*
* G. Sharon, R. Stern, A. Felner, N. Sturtevant (2015). *Conflict-Based Search for Optimal Multi-Agent Pathfinding.* AIJ.
* S. Russell, P. Norvig. *Artificial Intelligence: A Modern Approach* (agents, PEAS, environment types, search).
* M. Wooldridge. *An Introduction to MultiAgent Systems* (BDI, hybrid architectures, auctions).
* FIPA (2002). *ACL Message Structure Specification.*
