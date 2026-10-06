# Experiment results

Seeds 1..10; default scenario: 32x24 city, 2 hubs, 3 swap stations, 12 drones, 0.16 orders/tick for 500 ticks, gust p=0.03, 1 flight layer (the layer experiment varies it; the continuous-flight sections, tactical, motion and wind, use the default 3 layers). Times are in ticks (1 tick ~ 10 s).

### Coordination: How should drones avoid each other?

Mean ± standard deviation over 10 seeds.

| configuration | collisions | delivered | avg time | p95 time | on time | drones lost | energy/deliv | reactive holds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| cooperative (12 drones) | 0.00 ±0.00 | 100.0% ±0.0 | 34.8 ±6.8 | 70.5 ±13.4 | 99.0% ±1.9 | 0.00 ±0.00 | 51.1 ±3.9 | 4.30 ±2.50 |
| reactive only (12 drones) | 0.00 ±0.00 | 99.7% ±0.6 | 38.9 ±10.1 | 83.7 ±26.0 | 97.4% ±4.1 | 0.40 ±0.70 | 54.5 ±4.2 | 206 ±94 |
| none (12 drones) | 63.2 ±26.7 | 100.0% ±0.0 | 32.6 ±5.0 | 66.5 ±9.7 | 99.1% ±1.6 | 0.00 ±0.00 | 50.3 ±3.1 | 0.00 ±0.00 |
| cooperative (24 drones) | 0.00 ±0.00 | 100.0% ±0.0 | 57.9 ±24.6 | 142 ±71 | 86.8% ±11.7 | 0.00 ±0.00 | 55.4 ±4.8 | 18.5 ±4.2 |
| reactive only (24 drones) | 0.00 ±0.00 | 97.1% ±8.0 | 90.8 ±39.7 | 232 ±112 | 72.9% ±16.5 | 2.50 ±4.77 | 64.5 ±8.8 | 980 ±528 |
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
| 4 drones | 24.5 ±3.6 | 4.45 ±0.69 | 35.3 ±6.1 | 99.6% ±1.3 | 0.01 ±0.03 | 0.00 ±0.00 | 0.32 ±0.76 | 0.06 ±0.06 |
| 8 drones | 55.4 ±5.5 | 9.68 ±1.12 | 34.9 ±6.6 | 98.8% ±1.7 | 0.46 ±0.55 | 0.00 ±0.00 | 0.07 ±0.01 | 0.08 ±0.01 |
| 12 drones | 84.7 ±9.3 | 15.0 ±1.4 | 34.3 ±5.8 | 99.3% ±1.2 | 2.36 ±1.19 | 0.00 ±0.00 | 0.21 ±0.30 | 0.15 ±0.07 |
| 16 drones | 110 ±12 | 19.0 ±2.2 | 31.8 ±4.1 | 99.2% ±0.9 | 6.01 ±2.93 | 0.00 ±0.00 | 0.22 ±0.19 | 0.21 ±0.07 |
| 24 drones | 167 ±20 | 25.3 ±1.9 | 47.6 ±22.0 | 92.1% ±9.6 | 24.6 ±9.0 | 0.00 ±0.00 | 0.11 ±0.05 | 0.26 ±0.04 |
| 32 drones | 217 ±18 | 28.1 ±1.6 | 73.0 ±31.5 | 80.6% ±14.2 | 59.5 ±10.5 | 0.00 ±0.00 | 0.11 ±0.04 | 0.37 ±0.05 |

### Infrastructure: With 24 drones, what relieves the battery-swap bottleneck?

Mean ± standard deviation over 10 seeds.

| configuration | swap wait | max queue | avg time | p95 time | on time | deliv/100t |
|---|---:|---:|---:|---:|---:|---:|
| 1 bay, 3 spare packs | 28.9 ±6.7 | 7.00 ±1.49 | 57.9 ±24.6 | 142 ±71 | 86.8% ±11.7 | 26.1 ±2.3 |
| 1 bay, 6 spare packs | 0.85 ±0.38 | 2.30 ±0.82 | 29.6 ±3.1 | 55.4 ±9.3 | 99.9% ±0.2 | 31.0 ±2.7 |
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

### Layers: Do altitude layers add airspace capacity, and does a heading rule help?

Mean ± standard deviation over 10 seeds.

| configuration | collisions | avg time | p95 time | reactive holds | yields | energy/deliv | ms/A* | above layer 1 | NFZ violations |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 layer, free (24 drones) | 0.00 ±0.00 | 29.6 ±3.1 | 55.4 ±9.3 | 18.5 ±4.1 | 17.3 ±5.6 | 49.8 ±3.1 | 0.19 ±0.24 | 0.0% ±0.0 | 0.20 ±0.63 |
| 3 layers, free (24 drones) | 0.00 ±0.00 | 29.8 ±3.3 | 56.0 ±9.5 | 18.4 ±5.7 | 17.3 ±5.5 | 49.9 ±3.0 | 0.64 ±1.00 | 2.1% ±1.8 | 0.30 ±0.95 |
| 5 layers, free (24 drones) | 0.00 ±0.00 | 29.9 ±3.3 | 57.0 ±9.3 | 17.4 ±5.0 | 16.0 ±5.1 | 49.9 ±3.0 | 1.03 ±1.76 | 1.3% ±0.8 | 0.50 ±1.08 |
| 1 layer, heading (24 drones) | 0.00 ±0.00 | 29.6 ±3.1 | 55.4 ±9.3 | 18.5 ±4.1 | 17.3 ±5.6 | 49.8 ±3.1 | 0.19 ±0.24 | 0.0% ±0.0 | 0.20 ±0.63 |
| 3 layers, heading (24 drones) | 0.00 ±0.00 | 35.5 ±7.3 | 66.4 ±18.9 | 15.5 ±5.2 | 13.2 ±4.8 | 56.6 ±3.7 | 0.77 ±1.03 | 57.8% ±4.8 | 0.00 ±0.00 |
| 5 layers, heading (24 drones) | 0.00 ±0.00 | 36.7 ±8.2 | 69.1 ±19.5 | 17.6 ±6.5 | 16.3 ±6.8 | 57.1 ±3.6 | 1.31 ±2.38 | 57.5% ±4.9 | 0.00 ±0.00 |
| 1 layer, free (32 drones) | 0.00 ±0.00 | 31.5 ±5.4 | 59.8 ±14.2 | 36.4 ±12.1 | 39.8 ±15.1 | 51.4 ±3.5 | 0.25 ±0.27 | 0.0% ±0.0 | 0.30 ±0.95 |
| 3 layers, free (32 drones) | 0.00 ±0.00 | 30.5 ±5.3 | 58.2 ±13.7 | 31.5 ±5.9 | 33.5 ±7.8 | 50.9 ±3.4 | 0.78 ±1.10 | 2.5% ±1.7 | 0.40 ±0.97 |
| 5 layers, free (32 drones) | 0.00 ±0.00 | 31.2 ±5.2 | 60.3 ±12.9 | 31.8 ±8.0 | 32.6 ±9.0 | 51.2 ±3.4 | 1.24 ±1.90 | 1.8% ±1.1 | 0.60 ±1.26 |
| 1 layer, heading (32 drones) | 0.00 ±0.00 | 31.5 ±5.4 | 59.8 ±14.2 | 36.4 ±12.1 | 39.8 ±15.1 | 51.4 ±3.5 | 0.25 ±0.27 | 0.0% ±0.0 | 0.30 ±0.95 |
| 3 layers, heading (32 drones) | 0.00 ±0.00 | 41.1 ±10.2 | 79.9 ±21.6 | 26.9 ±8.3 | 25.6 ±6.7 | 58.8 ±4.3 | 0.98 ±0.94 | 58.5% ±3.9 | 0.00 ±0.00 |
| 5 layers, heading (32 drones) | 0.00 ±0.00 | 42.1 ±10.4 | 80.1 ±21.4 | 31.7 ±8.2 | 32.1 ±10.7 | 59.4 ±4.3 | 1.64 ±1.63 | 58.7% ±3.7 | 0.00 ±0.00 |

### Tactical: Continuous flight: what do strategic reservations and tactical ORCA each contribute?

Mean ± standard deviation over 10 seeds. Continuous flight (metres and seconds), default configuration with 3 flight layers. Wind presets: calm 0 m/s; moderate 5 m/s mean, 1.5 m/s RMS gusts; strong 8 m/s, 2.5 m/s. 24 drones: 0.35 orders/tick and 6 spare packs per station. LoS = loss of separation (closer than 40 m horizontally and 15 m vertically).

| configuration | collisions | LoS events | LoS pair-s | min sep (m) | avg time | on time | energy/deliv | wall s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| reservations only, calm wind (12 drones) | 0.00 ±0.00 | 28.0 ±8.2 | 109 ±40 | 18.6 ±2.1 | 31.2 ±4.5 | 99.3% ±1.2 | 50.5 ±3.3 | 0.51 ±0.16 |
| ORCA only, calm wind (12 drones) | 0.00 ±0.00 | 0.40 ±0.52 | 2.75 ±4.70 | 20.0 ±10.3 | 31.6 ±5.2 | 99.2% ±0.9 | 50.6 ±3.7 | 0.58 ±0.13 |
| reservations + ORCA, calm wind (12 drones) | 0.00 ±0.00 | 0.10 ±0.32 | 0.10 ±0.32 | 25.1 ±3.3 | 31.7 ±4.7 | 99.2% ±1.2 | 50.9 ±3.2 | 0.62 ±0.17 |
| reservations only, moderate wind (12 drones) | 0.00 ±0.00 | 27.5 ±6.6 | 113 ±32 | 18.6 ±2.0 | 31.7 ±4.1 | 99.3% ±0.7 | 58.4 ±4.6 | 0.78 ±0.56 |
| ORCA only, moderate wind (12 drones) | 0.10 ±0.32 | 0.80 ±0.92 | 8.00 ±13.20 | 15.8 ±13.0 | 31.1 ±4.4 | 99.5% ±0.6 | 58.2 ±4.3 | 0.83 ±0.50 |
| reservations + ORCA, moderate wind (12 drones) | 0.00 ±0.00 | 0.00 ±0.00 | 0.00 ±0.00 | 25.1 ±4.0 | 32.5 ±5.1 | 99.3% ±0.8 | 59.1 ±4.0 | 0.90 ±0.59 |
| reservations only, strong wind (12 drones) | 0.00 ±0.00 | 33.4 ±8.2 | 133 ±29 | 18.6 ±1.7 | 29.1 ±8.2 | 99.2% ±1.3 | 67.6 ±7.4 | 0.56 ±0.17 |
| ORCA only, strong wind (12 drones) | 0.00 ±0.00 | 0.60 ±0.84 | 3.85 ±6.16 | 18.3 ±12.5 | 29.8 ±9.1 | 99.0% ±1.6 | 68.6 ±7.2 | 0.65 ±0.16 |
| reservations + ORCA, strong wind (12 drones) | 0.00 ±0.00 | 0.10 ±0.32 | 0.20 ±0.63 | 23.4 ±1.9 | 28.5 ±6.7 | 99.1% ±1.6 | 68.1 ±6.7 | 0.66 ±0.17 |
| reservations only, calm wind (24 drones) | 0.00 ±0.00 | 123 ±32 | 501 ±136 | 15.5 ±1.5 | 28.3 ±4.0 | 99.8% ±0.4 | 49.4 ±3.1 | 1.17 ±0.50 |
| ORCA only, calm wind (24 drones) | 0.30 ±0.48 | 4.20 ±3.05 | 27.6 ±27.4 | 7.33 ±10.15 | 28.2 ±3.3 | 99.9% ±0.2 | 50.4 ±2.7 | 1.61 ±0.55 |
| reservations + ORCA, calm wind (24 drones) | 0.00 ±0.00 | 0.30 ±0.48 | 0.40 ±0.74 | 22.4 ±0.6 | 29.4 ±3.3 | 99.9% ±0.2 | 50.9 ±3.0 | 1.70 ±0.57 |
| reservations only, moderate wind (24 drones) | 0.00 ±0.00 | 126 ±27 | 500 ±112 | 16.4 ±1.9 | 28.7 ±3.4 | 99.9% ±0.2 | 58.1 ±3.9 | 1.44 ±0.52 |
| ORCA only, moderate wind (24 drones) | 0.20 ±0.42 | 4.10 ±2.42 | 23.9 ±17.9 | 6.79 ±9.23 | 28.0 ±2.8 | 99.9% ±0.2 | 59.1 ±3.3 | 1.75 ±0.44 |
| reservations + ORCA, moderate wind (24 drones) | 0.00 ±0.00 | 0.80 ±0.79 | 1.05 ±1.32 | 22.2 ±0.6 | 29.9 ±3.6 | 99.9% ±0.2 | 60.0 ±3.7 | 1.93 ±0.51 |
| reservations only, strong wind (24 drones) | 0.00 ±0.00 | 149 ±29 | 598 ±119 | 16.2 ±3.7 | 27.3 ±4.3 | 99.8% ±0.3 | 66.5 ±4.3 | 1.26 ±0.63 |
| ORCA only, strong wind (24 drones) | 0.00 ±0.00 | 4.40 ±2.01 | 27.2 ±15.8 | 3.78 ±6.57 | 26.5 ±4.2 | 99.9% ±0.2 | 67.3 ±4.1 | 1.60 ±0.44 |
| reservations + ORCA, strong wind (24 drones) | 0.00 ±0.00 | 0.60 ±0.70 | 0.75 ±1.11 | 22.1 ±0.4 | 30.8 ±7.4 | 99.4% ±1.0 | 69.8 ±4.8 | 1.66 ±0.28 |

### Motion: Grid cells or continuous flight, at the default configuration?

Mean ± standard deviation over 10 seeds. Default configuration (12 drones, 3 flight layers, 0.16 orders/tick); 32 drones fly with 0.47 orders/tick and 6 spare packs per station. The grid model's wind is a 3 % chance per move of being held back a tick; continuous flight uses the wind field. "Cruise at 60 m" (cruise_layer = 2, the default of run_simulation.py --motion continuous): drones prefer to fly 2 layers above the street or roof below them and close to the straight line to their goal, so they cruise at 60 m and climb to 90 m over buildings; "90 m" is cruise_layer = 3. "Over roofs at 90 m": share of the time spent over a building that is flown at 90 m.

| configuration | delivered | avg time | p95 time | on time | energy/deliv | above layer 1 | over roofs at 90 m | collisions | wall s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| grid (default) | 100.0% ±0.0 | 34.6 ±6.9 | 70.4 ±15.9 | 98.8% ±2.0 | 51.4 ±3.4 | 2.1% ±2.3 | 19.1% ±33.1 | 0.00 ±0.00 | 0.23 ±0.29 |
| continuous, calm | 100.0% ±0.0 | 31.7 ±4.7 | 63.9 ±9.7 | 99.2% ±1.2 | 50.9 ±3.2 | 2.6% ±2.6 | 20.8% ±32.8 | 0.00 ±0.00 | 0.61 ±0.17 |
| continuous, moderate wind (default) | 100.0% ±0.0 | 32.5 ±5.1 | 68.8 ±11.2 | 99.3% ±0.8 | 59.1 ±4.0 | 2.1% ±2.4 | 17.5% ±36.8 | 0.00 ±0.00 | 0.90 ±0.59 |
| grid, 32 drones | 100.0% ±0.0 | 30.5 ±5.3 | 58.2 ±13.7 | 99.5% ±0.8 | 50.9 ±3.4 | 2.5% ±1.7 | 11.4% ±24.8 | 0.00 ±0.00 | 0.93 ±0.86 |
| continuous, moderate wind, 32 drones | 100.0% ±0.0 | 34.2 ±6.4 | 66.5 ±19.6 | 98.8% ±2.1 | 63.8 ±2.4 | 4.9% ±1.8 | 23.6% ±24.2 | 0.00 ±0.00 | 3.04 ±0.55 |
| continuous, cruise at 60 m | 100.0% ±0.0 | 42.3 ±11.9 | 86.2 ±26.4 | 97.4% ±3.9 | 66.0 ±4.4 | 87.0% ±0.6 | 81.6% ±4.2 | 0.00 ±0.00 | 0.93 ±0.26 |
| continuous, cruise at 60 m, 32 drones | 100.0% ±0.0 | 43.6 ±11.6 | 88.0 ±31.5 | 96.4% ±5.1 | 68.5 ±4.3 | 86.7% ±0.6 | 82.3% ±4.4 | 0.00 ±0.00 | 3.31 ±0.61 |
| continuous, cruise at 90 m | 100.0% ±0.0 | 58.5 ±20.9 | 113 ±39 | 89.0% ±15.2 | 79.5 ±5.8 | 88.6% ±0.6 | 100.0% ±0.1 | 0.00 ±0.00 | 1.09 ±0.35 |

### Wind: Continuous flight: how much wind can the energy-safe fleet fly in?

Mean ± standard deviation over 10 seeds. Reservations + ORCA, default configuration (12 drones). Mean wind at 30 m and RMS gusts: calm 0/0, moderate 5/1.5, strong 8/2.5, severe 10/3 m/s.

| configuration | delivered | avg time | energy/deliv | flight h | LoS events | min sep (m) | track err (m) | emergencies |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| calm wind | 100.0% ±0.0 | 31.7 ±4.7 | 50.9 ±3.2 | 10.9 ±1.5 | 0.10 ±0.32 | 25.1 ±3.3 | 3.07 ±0.17 | 0.10 ±0.32 |
| moderate wind | 100.0% ±0.0 | 32.5 ±5.1 | 59.1 ±4.0 | 11.0 ±1.5 | 0.00 ±0.00 | 25.1 ±4.0 | 3.16 ±0.24 | 0.20 ±0.63 |
| strong wind | 100.0% ±0.0 | 28.5 ±6.7 | 68.1 ±6.7 | 10.3 ±1.4 | 0.10 ±0.32 | 23.4 ±1.9 | 3.91 ±0.33 | 0.10 ±0.32 |
| severe wind | 20.6% ±12.0 | 93.8 ±73.1 | 49.3 ±10.6 | 1.28 ±0.79 | 0.00 ±0.00 | 65.2 ±33.8 | 6.14 ±1.07 | 0.00 ±0.00 |

_Total compute: 515.2 s._
