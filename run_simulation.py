#!/usr/bin/env python3
"""Run one drone-fleet simulation and export an interactive HTML replay.

Examples
--------
    python3 run_simulation.py                              # default scenario
    python3 run_simulation.py --drones 20 --rate 0.3       # busier city
    python3 run_simulation.py --coordination none          # watch the collisions
    python3 run_simulation.py --battery naive --seed 1     # watch drones run dry
    python3 run_simulation.py --layers 5 --layer-rule heading   # stacked airspace
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from pathlib import Path

from dronefleet import SimConfig, Simulation
from dronefleet.config import ALLOCATION_STRATEGIES, BATTERY_POLICIES, COORDINATION_MODES, LAYER_RULES
from dronefleet.metrics import format_metrics
from dronefleet.replay import export_html


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--drones", type=int, default=SimConfig.n_drones)
    ap.add_argument("--rate", type=float, default=SimConfig.order_rate, help="orders per tick (Poisson mean)")
    ap.add_argument("--ticks", type=int, default=SimConfig.max_ticks)
    ap.add_argument("--allocation", choices=ALLOCATION_STRATEGIES, default="cnp")
    ap.add_argument("--coordination", choices=COORDINATION_MODES, default="cooperative")
    ap.add_argument("--battery", choices=BATTERY_POLICIES, default="predictive")
    ap.add_argument("--gust", type=float, default=SimConfig.gust_prob, help="wind-gust probability per move")
    ap.add_argument("--layers", type=int, default=SimConfig.n_layers, help="flight layers (1 = flat airspace)")
    ap.add_argument("--layer-rule", choices=LAYER_RULES, default=SimConfig.layer_rule,
                    help="heading: east/west traffic on odd layers, north/south on even layers")
    ap.add_argument("--no-nfz", action="store_true", help="disable temporary no-fly zones")
    ap.add_argument("--out", default="results/replay.html", help="HTML replay path ('' to skip)")
    ap.add_argument("--metrics-json", default="", help="also write metrics to this JSON file")
    ap.add_argument("--map", action="store_true", help="print the ASCII city map")
    ap.add_argument("--events", type=int, default=0, help="print the first N event-log lines")
    ap.add_argument("--quiet", action="store_true")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    cfg = dataclasses.replace(
        SimConfig(), seed=a.seed, n_drones=a.drones, order_rate=a.rate, max_ticks=a.ticks,
        allocation=a.allocation, coordination=a.coordination, battery_policy=a.battery,
        gust_prob=a.gust, auto_nfz=not a.no_nfz, n_layers=a.layers, layer_rule=a.layer_rule,
        record_trace=bool(a.out),
    )
    sim = Simulation(cfg)
    w = sim.world
    if not a.quiet:
        print(f"City {w.width}x{w.height}: {len(w.hubs)} hubs, {len(w.stations)} swap stations, "
              f"{len(w.customers)} customer sites, {len(w.nfzs)} scheduled no-fly zones")
        print(f"Fleet of {cfg.n_drones} drones | allocation={cfg.allocation} "
              f"coordination={cfg.coordination} battery={cfg.battery_policy} gust={cfg.gust_prob} "
              f"layers={cfg.n_layers} rule={w.layer_rule}")
    if a.map:
        print(w.ascii())
        print("legend: H hub  S swap station  c customer  "
              + ("# building" if w.n_layers == 1 else "1-9 building height in layers"))
    metrics = sim.run(progress=not a.quiet)
    if a.events:
        for t, e in sim.events[: a.events]:
            print(f"[t={t:4d}] {e}")
    print("\nResults")
    print(format_metrics(metrics))
    if a.out:
        path = export_html(sim, metrics, a.out)
        print(f"\nReplay written to {path}  (open it in a browser)")
    if a.metrics_json:
        Path(a.metrics_json).parent.mkdir(parents=True, exist_ok=True)
        Path(a.metrics_json).write_text(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
