#!/usr/bin/env python3
"""Controlled experiments comparing coordination, allocation and battery strategies.

Every configuration is run on the same set of random seeds (same cities, same
order streams, same gusts) so differences come from the strategy alone.
Results are written as Markdown tables + JSON, plus SVG charts for the report.

    python3 run_experiments.py              # 10 seeds, a few minutes
    python3 run_experiments.py --seeds 3    # quick look
    python3 run_experiments.py --only layers
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import statistics
import sys
import time
from pathlib import Path

from dronefleet import SimConfig, Simulation

# The original experiments use one flight layer: with n_layers=1 the simulator
# reproduces the flat-airspace study exactly (tests/test_regression.py).
BASE = SimConfig(record_trace=False, n_layers=1)
DENSE = dict(n_drones=24, order_rate=0.35)
# Altitude-layer experiment: dense fleets with the same demand per drone, and
# 6 spare packs per station so the battery-swap queue (see "infrastructure")
# does not mask what happens in the airspace.
LAYER_FLEETS = ((24, 0.35), (32, 0.47))
LAYER_INFRA = dict(station_spare_packs=6)


def layer_label(layers: int, rule: str, drones: int) -> str:
    return f"{layers} layer{'s' if layers > 1 else ''}, {rule} ({drones} drones)"


def experiments() -> dict[str, dict]:
    """name -> {question, variants: [(label, overrides)], metrics: [...]}"""
    return {
        "coordination": {
            "question": "How should drones avoid each other?",
            "variants": [
                ("cooperative (12 drones)", dict(coordination="cooperative")),
                ("reactive only (12 drones)", dict(coordination="reactive")),
                ("none (12 drones)", dict(coordination="none")),
                ("cooperative (24 drones)", dict(coordination="cooperative", **DENSE)),
                ("reactive only (24 drones)", dict(coordination="reactive", **DENSE)),
                ("none (24 drones)", dict(coordination="none", **DENSE)),
            ],
            "metrics": ["collisions", "delivery_rate", "avg_delivery_time", "p95_delivery_time",
                        "on_time_rate", "dead_drones", "energy_per_delivery", "forced_holds"],
        },
        "allocation": {
            "question": "Who should deliver which parcel?",
            "variants": [
                ("contract net (12 drones)", dict(allocation="cnp")),
                ("nearest idle (12 drones)", dict(allocation="nearest")),
                ("round robin (12 drones)", dict(allocation="round_robin")),
                ("contract net (24 drones)", dict(allocation="cnp", **DENSE)),
                ("nearest idle (24 drones)", dict(allocation="nearest", **DENSE)),
                ("round robin (24 drones)", dict(allocation="round_robin", **DENSE)),
            ],
            "metrics": ["avg_delivery_time", "p95_delivery_time", "avg_express_time", "on_time_rate",
                        "energy_per_delivery", "swaps", "utilisation", "messages_excl_telemetry"],
        },
        "battery": {
            "question": "When should a drone swap its battery?",
            "variants": [
                ("predictive (energy-aware)", dict(battery_policy="predictive")),
                ("naive 30% threshold", dict(battery_policy="naive")),
                ("predictive, 24 drones", dict(battery_policy="predictive", **DENSE)),
                ("naive, 24 drones", dict(battery_policy="naive", **DENSE)),
            ],
            "metrics": ["dead_drones", "failed", "delivery_rate", "emergencies", "swaps",
                        "avg_swap_wait", "avg_delivery_time", "energy_per_delivery"],
        },
        "scalability": {
            "question": "How does the system scale with fleet size (demand scaled with the fleet)?",
            "variants": [(f"{n} drones", dict(n_drones=n, order_rate=round(0.0135 * n, 4)))
                         for n in (4, 8, 12, 16, 24, 32)],
            "metrics": ["orders", "throughput_per_100t", "avg_delivery_time", "on_time_rate",
                        "avg_swap_wait", "collisions", "planner_ms_per_search", "wall_time_s"],
        },
        "infrastructure": {
            "question": "With 24 drones, what relieves the battery-swap bottleneck?",
            "variants": [(f"{b} bay{'s' if b > 1 else ''}, {p} spare packs",
                          dict(station_bays=b, station_spare_packs=p, **DENSE))
                         for b in (1, 2) for p in (3, 6)],
            "metrics": ["avg_swap_wait", "max_station_queue", "avg_delivery_time", "p95_delivery_time",
                        "on_time_rate", "throughput_per_100t"],
        },
        "robustness": {
            "question": "Does coordination survive execution noise (wind gusts)?",
            "variants": [(f"gust p={p}", dict(gust_prob=p)) for p in (0.0, 0.05, 0.1, 0.2, 0.3)],
            "metrics": ["collisions", "deviations", "plan_repairs", "replans", "yields",
                        "avg_delivery_time", "on_time_rate", "energy_per_delivery"],
        },
        "layers": {
            "question": "Do altitude layers add airspace capacity, and does a heading rule help?",
            "variants": [(layer_label(layers, rule, n),
                          dict(n_layers=layers, layer_rule=rule, n_drones=n, order_rate=rate, **LAYER_INFRA))
                         for n, rate in LAYER_FLEETS for rule in ("free", "heading") for layers in (1, 3, 5)],
            "metrics": ["collisions", "avg_delivery_time", "p95_delivery_time", "forced_holds", "yields",
                        "energy_per_delivery", "planner_ms_per_search", "upper_layer_share", "nfz_violations"],
        },
    }


def run_variant(overrides: dict, seeds: list[int]) -> list[dict]:
    out = []
    for s in seeds:
        sim = Simulation(dataclasses.replace(BASE, seed=s, **overrides))
        out.append(sim.run())
    return out


def summarise(runs: list[dict], keys: list[str]) -> dict:
    res = {}
    for k in keys:
        vals = [float(r[k]) for r in runs]
        res[k] = {"mean": statistics.fmean(vals), "std": statistics.stdev(vals) if len(vals) > 1 else 0.0,
                  "min": min(vals), "max": max(vals)}
    return res


LABELS = {
    "collisions": "collisions", "delivery_rate": "delivered", "avg_delivery_time": "avg time",
    "p95_delivery_time": "p95 time", "on_time_rate": "on time", "dead_drones": "drones lost",
    "energy_per_delivery": "energy/deliv", "forced_holds": "reactive holds",
    "avg_express_time": "express avg", "swaps": "swaps", "utilisation": "utilisation",
    "messages_excl_telemetry": "msgs (no telem.)", "failed": "orders lost", "emergencies": "emergencies",
    "avg_swap_wait": "swap wait", "orders": "orders", "throughput_per_100t": "deliv/100t",
    "replans": "plans", "planner_ms_per_search": "ms/A*", "wall_time_s": "wall s",
    "deviations": "deviations", "plan_repairs": "repairs", "yields": "yields",
    "max_station_queue": "max queue", "upper_layer_share": "above layer 1", "nfz_violations": "NFZ violations",
}
PERCENT = {"delivery_rate", "on_time_rate", "utilisation", "upper_layer_share"}


def fmt(k: str, s: dict) -> str:
    m, sd = s["mean"], s["std"]
    if k in PERCENT:
        return f"{100 * m:.1f}% ±{100 * sd:.1f}"
    if abs(m) >= 100:
        return f"{m:.0f} ±{sd:.0f}"
    return f"{m:.2f} ±{sd:.2f}" if abs(m) < 10 else f"{m:.1f} ±{sd:.1f}"


def to_markdown(name: str, exp: dict, rows: list[tuple[str, dict]], n_seeds: int) -> str:
    keys = exp["metrics"]
    lines = [f"### {name.capitalize()}: {exp['question']}", "",
             f"Mean ± standard deviation over {n_seeds} seeds.", "",
             "| configuration | " + " | ".join(LABELS.get(k, k) for k in keys) + " |",
             "|---|" + "---:|" * len(keys)]
    for label, summ in rows:
        lines.append(f"| {label} | " + " | ".join(fmt(k, summ[k]) for k in keys) + " |")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--only", nargs="*", help="run only these experiments")
    ap.add_argument("--out", default="results")
    ap.add_argument("--no-charts", action="store_true")
    a = ap.parse_args(argv)

    seeds = list(range(1, a.seeds + 1))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    md = ["# Experiment results", "",
          f"Seeds {seeds[0]}..{seeds[-1]}; default scenario: {BASE.width}x{BASE.height} city, "
          f"{BASE.n_hubs} hubs, {BASE.n_stations} swap stations, {BASE.n_drones} drones, "
          f"{BASE.order_rate} orders/tick for {BASE.order_until} ticks, gust p={BASE.gust_prob}, "
          f"{BASE.n_layers} flight layer (the layer experiment varies it). "
          f"Times are in ticks (1 tick ~ 10 s).", ""]
    all_results: dict = {}
    t0 = time.perf_counter()
    for name, exp in experiments().items():
        if a.only and name not in a.only:
            continue
        print(f"== {name}: {exp['question']}", flush=True)
        rows = []
        all_results[name] = {"question": exp["question"], "variants": []}
        for label, ov in exp["variants"]:
            runs = run_variant(ov, seeds)
            summ = summarise(runs, sorted(set(exp["metrics"]) | {"collisions", "delivery_rate",
                                                                  "avg_delivery_time", "dead_drones"}))
            rows.append((label, summ))
            all_results[name]["variants"].append({"label": label, "overrides": ov, "summary": summ,
                                                  "runs": runs})
            print(f"   {label:30s} " + "  ".join(f"{LABELS.get(k, k)}={fmt(k, summ[k])}"
                                                  for k in exp["metrics"][:5]), flush=True)
        md.append(to_markdown(name, exp, rows, len(seeds)))
    md.append(f"_Total compute: {time.perf_counter() - t0:.1f} s._\n")
    (out / "experiments.md").write_text("\n".join(md))
    (out / "experiments.json").write_text(json.dumps(all_results, indent=1))
    print(f"\nWrote {out / 'experiments.md'} and {out / 'experiments.json'}")
    if not a.no_charts:
        from dronefleet.charts import write_charts
        for p in write_charts(all_results, out / "figures"):
            print(f"chart: {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
