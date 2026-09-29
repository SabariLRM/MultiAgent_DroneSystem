# Experiment results

Seeds 1..10; default scenario: 32x24 city, 2 hubs, 3 swap stations, 12 drones, 0.16 orders/tick for 500 ticks, gust p=0.03. Times are in ticks (1 tick ~ 10 s).

### Coordination: How should drones avoid each other?

Mean ± standard deviation over 10 seeds.

| configuration | collisions | delivered | avg time | p95 time | on time | drones lost | energy/deliv | reactive holds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| cooperative (12 drones) | 0.00 ±0.00 | 100.0% ±0.0 | 34.8 ±6.8 | 70.5 ±13.4 | 99.0% ±1.9 | 0.00 ±0.00 | 51.1 ±3.9 | 4.30 ±2.50 |
| reactive only (12 drones) | 0.00 ±0.00 | 99.7% ±0.6 | 38.9 ±10.1 | 83.7 ±26.0 | 97.4% ±4.1 | 0.40 ±0.70 | 54.5 ±4.2 | 206 ±94 |
| none (12 drones) | 63.2 ±26.7 | 100.0% ±0.0 | 32.6 ±5.0 | 66.5 ±9.7 | 99.1% ±1.6 | 0.00 ±0.00 | 50.3 ±3.1 | 0.00 ±0.00 |
| cooperative (24 drones) | 0.00 ±0.00 | 100.0% ±0.0 | 57.9 ±24.6 | 142 ±71 | 86.8% ±11.7 | 0.00 ±0.00 | 55.4 ±4.8 | 18.5 ±4.2 |
| reactive only (24 drones) | 0.00 ±0.00 | 97.1% ±8.0 | 90.6 ±39.7 | 230 ±112 | 73.1% ±16.5 | 2.50 ±4.77 | 64.4 ±8.9 | 970 ±536 |
| none (24 drones) | 268 ±86 | 100.0% ±0.0 | 49.9 ±20.8 | 123 ±68 | 90.9% ±9.0 | 0.00 ±0.00 | 53.7 ±4.5 | 0.00 ±0.00 |

### Allocation: Who should deliver which parcel?

Mean ± standard deviation over 10 seeds.

| configuration | avg time | p95 time | express avg | on time | energy/deliv | swaps | utilisation | msgs (no telem.) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| contract net (12 drones) | 34.8 ±6.8 | 70.5 ±13.4 | 32.1 ±7.6 | 99.0% ±1.9 | 51.1 ±3.9 | 44.4 ±8.2 | 60.0% ±8.5 | 1517 ±148 |
| nearest idle (12 drones) | 34.5 ±7.5 | 75.7 ±18.3 | 30.5 ±6.0 | 98.9% ±1.6 | 52.1 ±3.8 | 46.2 ±7.6 | 61.4% ±7.7 | 575 ±72 |
| round robin (12 drones) | 51.3 ±7.8 | 91.5 ±13.0 | 45.8 ±7.1 | 97.1% ±1.7 | 62.8 ±4.2 | 58.0 ±4.3 | 74.3% ±5.8 | 634 ±56 |
| contract net (24 drones) | 57.9 ±24.6 | 142 ±71 | 46.1 ±14.1 | 86.8% ±11.7 | 55.4 ±4.8 | 101 ±19 | 74.0% ±6.2 | 4224 ±767 |
| nearest idle (24 drones) | 61.6 ±20.6 | 140 ±48 | 40.5 ±5.9 | 89.4% ±11.3 | 57.2 ±3.7 | 108 ±15 | 76.5% ±4.4 | 1272 ±134 |
| round robin (24 drones) | 102 ±28 | 203 ±52 | 51.1 ±4.2 | 69.4% ±17.5 | 64.0 ±4.1 | 123 ±15 | 80.7% ±2.9 | 1347 ±131 |

### Battery: When should a drone swap its battery?

Mean ± standard deviation over 10 seeds.

| configuration | drones lost | orders lost | delivered | emergencies | swaps | swap wait | avg time | energy/deliv |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| predictive (energy-aware) | 0.00 ±0.00 | 0.00 ±0.00 | 100.0% ±0.0 | 0.20 ±0.63 | 44.4 ±8.2 | 2.68 ±1.90 | 34.8 ±6.8 | 51.1 ±3.9 |
| naive 30% threshold | 4.80 ±3.05 | 2.20 ±1.62 | 97.4% ±2.0 | 6.30 ±3.13 | 27.8 ±4.3 | 2.73 ±3.09 | 47.2 ±20.1 | 52.2 ±4.1 |
| predictive, 24 drones | 0.00 ±0.00 | 0.00 ±0.00 | 100.0% ±0.0 | 0.10 ±0.32 | 101 ±19 | 28.9 ±6.7 | 57.9 ±24.6 | 55.4 ±4.8 |
| naive, 24 drones | 8.60 ±3.50 | 4.70 ±3.09 | 97.4% ±1.6 | 11.5 ±6.1 | 60.7 ±7.2 | 20.2 ±5.2 | 44.9 ±21.3 | 51.9 ±3.9 |

### Scalability: How does the system scale with fleet size (demand scaled with the fleet)?

Mean ± standard deviation over 10 seeds.

| configuration | orders | deliv/100t | avg time | on time | swap wait | collisions | ms/A* | wall s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 4 drones | 24.5 ±3.6 | 4.45 ±0.69 | 35.3 ±6.1 | 99.6% ±1.3 | 0.01 ±0.03 | 0.00 ±0.00 | 0.31 ±0.75 | 0.06 ±0.06 |
| 8 drones | 55.4 ±5.5 | 9.68 ±1.12 | 34.9 ±6.6 | 98.8% ±1.7 | 0.46 ±0.55 | 0.00 ±0.00 | 0.07 ±0.01 | 0.08 ±0.01 |
| 12 drones | 84.7 ±9.3 | 15.0 ±1.4 | 34.3 ±5.8 | 99.3% ±1.2 | 2.36 ±1.19 | 0.00 ±0.00 | 0.20 ±0.30 | 0.14 ±0.07 |
| 16 drones | 110 ±12 | 19.0 ±2.2 | 31.8 ±4.1 | 99.2% ±0.9 | 6.01 ±2.93 | 0.00 ±0.00 | 0.22 ±0.18 | 0.20 ±0.07 |
| 24 drones | 167 ±20 | 25.3 ±1.9 | 47.6 ±22.0 | 92.1% ±9.6 | 24.6 ±9.0 | 0.00 ±0.00 | 0.10 ±0.05 | 0.24 ±0.03 |
| 32 drones | 217 ±18 | 28.1 ±1.6 | 73.0 ±31.5 | 80.6% ±14.2 | 59.5 ±10.5 | 0.00 ±0.00 | 0.11 ±0.04 | 0.35 ±0.05 |

### Infrastructure: With 24 drones, what relieves the battery-swap bottleneck?

Mean ± standard deviation over 10 seeds.

| configuration | swap wait | max queue | avg time | p95 time | on time | deliv/100t |
|---|---:|---:|---:|---:|---:|---:|
| 1 bay, 3 spare packs | 28.9 ±6.7 | 7.00 ±1.49 | 57.9 ±24.6 | 142 ±71 | 86.8% ±11.7 | 26.1 ±2.3 |
| 1 bay, 6 spare packs | 0.89 ±0.43 | 2.50 ±0.97 | 29.5 ±3.1 | 55.4 ±9.3 | 99.9% ±0.2 | 31.0 ±2.7 |
| 2 bays, 3 spare packs | 28.8 ±5.5 | 7.30 ±0.82 | 55.9 ±22.9 | 135 ±66 | 88.6% ±10.0 | 26.0 ±1.5 |
| 2 bays, 6 spare packs | 1.40 ±0.49 | 3.60 ±0.97 | 30.1 ±4.0 | 56.4 ±10.4 | 99.8% ±0.3 | 31.2 ±2.3 |

### Robustness: Does coordination survive execution noise (wind gusts)?

Mean ± standard deviation over 10 seeds.

| configuration | collisions | deviations | repairs | plans | yields | avg time | on time | energy/deliv |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| gust p=0.0 | 0.00 ±0.00 | 0.00 ±0.00 | 0.00 ±0.00 | 66.0 ±14.5 | 0.00 ±0.00 | 32.0 ±4.7 | 99.1% ±1.6 | 49.6 ±3.5 |
| gust p=0.05 | 0.00 ±0.00 | 186 ±27 | 164 ±23 | 89.4 ±19.7 | 5.60 ±2.67 | 34.8 ±6.2 | 99.0% ±1.5 | 51.9 ±3.1 |
| gust p=0.1 | 0.00 ±0.00 | 388 ±67 | 342 ±52 | 121 ±37 | 10.3 ±7.7 | 39.3 ±10.3 | 97.9% ±3.3 | 54.6 ±4.3 |
| gust p=0.2 | 0.00 ±0.00 | 914 ±122 | 803 ±111 | 196 ±40 | 21.1 ±9.0 | 49.4 ±15.4 | 94.8% ±7.7 | 61.0 ±4.3 |
| gust p=0.3 | 0.00 ±0.00 | 1621 ±256 | 1434 ±219 | 287 ±66 | 45.2 ±21.0 | 67.7 ±24.2 | 85.4% ±16.1 | 69.0 ±4.9 |

_Total compute: 64.6 s._
