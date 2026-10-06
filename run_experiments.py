#!/usr/bin/env python3
"""Controlled experiments comparing coordination, allocation and battery strategies.

Every configuration is run on the same set of random seeds (same cities, same
order streams, same gusts) so differences come from the strategy alone.
Results are written as Markdown tables + JSON, plus SVG charts for the report.

    python3 run_experiments.py              # 10 seeds, about 15 minutes
    python3 run_experiments.py --seeds 3    # quick look
    python3 run_experiments.py --only layers
    python3 run_experiments.py --only tactical motion wind --jobs 8

With ``--only`` the named sections replace their old results in
``experiments.md`` / ``experiments.json`` and every other section is kept as
it was, so one experiment can be re-run without re-running (and re-timing)
the rest. Only the charts of the experiments that ran are rewritten.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import re
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from dronefleet import SimConfig, Simulation
from dronefleet.wind import WIND_PRESETS

# The original experiments use one flight layer: with n_layers=1 the simulator
# reproduces the flat-airspace study exactly (tests/test_regression.py).
BASE = SimConfig(record_trace=False, n_layers=1)
DENSE = dict(n_drones=24, order_rate=0.35)
# Altitude-layer experiment: dense fleets with the same demand per drone, and
# 6 spare packs per station so the battery-swap queue (see "infrastructure")
# does not mask what happens in the airspace.
LAYER_FLEETS = ((24, 0.35), (32, 0.47))
LAYER_INFRA = dict(station_spare_packs=6)
# Continuous flight (docs/continuous_design.md) uses the default configuration,
# 3 flight layers. As in the layer experiment, 24 drones get 6 spare packs per
# station so the battery-swap queue does not mask what happens in the air.
CONT = dict(motion="continuous", n_layers=3)
TACTICAL = (("reservations only", dict(tactical="none")),
            ("ORCA only", dict(coordination="none")),
            ("reservations + ORCA", dict()))
TACTICAL_FLEETS = ((12, dict()), (24, dict(n_drones=24, order_rate=0.35, **LAYER_INFRA)))
TACTICAL_WINDS = ("calm", "moderate", "strong")
FLEET32 = dict(n_drones=32, order_rate=0.47, **LAYER_INFRA)


def wind(name: str) -> dict:
    mean, gust = WIND_PRESETS[name]
    return dict(wind_mean=mean, wind_gust=gust)


def tactical_label(strategy: str, wind_name: str, drones: int) -> str:
    return f"{strategy}, {wind_name} wind ({drones} drones)"


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
        "tactical": {
            "question": "Continuous flight: what do strategic reservations and tactical ORCA each contribute?",
            "note": "Continuous flight (metres and seconds), default configuration with 3 flight layers. "
                    "Wind presets: calm 0 m/s; moderate 5 m/s mean, 1.5 m/s RMS gusts; strong 8 m/s, 2.5 m/s. "
                    "24 drones: 0.35 orders/tick and 6 spare packs per station. "
                    "LoS = loss of separation (closer than 40 m horizontally and 15 m vertically).",
            "variants": [(tactical_label(strategy, w, n), dict(**CONT, **fleet, **wind(w), **ov))
                         for n, fleet in TACTICAL_FLEETS for w in TACTICAL_WINDS for strategy, ov in TACTICAL],
            "metrics": ["collisions", "separation_losses", "separation_loss_s", "min_separation_m",
                        "avg_delivery_time", "on_time_rate", "energy_per_delivery", "wall_time_s"],
            "extra": ["delivery_rate", "delivered", "orders", "failed", "orca_per_drone_hour", "tracking_error_mean_m",
                      "tracking_error_p95_m", "building_intrusions", "stall_replans", "emergencies", "flight_hours",
                      "p95_delivery_time", "swaps"],
        },
        "motion": {
            "question": "Grid cells or continuous flight, at the default configuration?",
            "note": "Default configuration (12 drones, 3 flight layers, 0.16 orders/tick); 32 drones fly with "
                    "0.47 orders/tick and 6 spare packs per station. The grid model's wind is a 3 % chance per "
                    "move of being held back a tick; continuous flight uses the wind field. \"Cruise at 60 m\" "
                    "(cruise_layer = 2, the default of run_simulation.py --motion continuous): drones prefer to fly "
                    "2 layers above the street or roof below them and close to the straight line to their goal, so "
                    "they cruise at 60 m and climb to 90 m over buildings; \"90 m\" is cruise_layer = 3. "
                    "\"Over roofs at 90 m\": share of the time spent over a building that is flown at 90 m. "
                    "\"Customers in buildings\" (the default of run_simulation.py --motion continuous): customers "
                    "live in houses (parcel lowered into the garden from 30 m) or buildings with a roof at 30 or 60 m "
                    "(parcel winched onto the roof from 60 or 90 m).",
            "variants": [("grid (default)", dict(n_layers=3)),
                         ("continuous, calm", dict(**CONT, **wind("calm"))),
                         ("continuous, moderate wind (default)", dict(**CONT)),
                         ("grid, 32 drones", dict(n_layers=3, **FLEET32)),
                         ("continuous, moderate wind, 32 drones", dict(**CONT, **FLEET32)),
                         ("continuous, cruise at 60 m", dict(**CONT, cruise_layer=2)),
                         ("continuous, cruise at 60 m, 32 drones", dict(**CONT, **FLEET32, cruise_layer=2)),
                         ("continuous, cruise at 90 m", dict(**CONT, cruise_layer=3)),
                         ("continuous, 60 m, customers in buildings", dict(**CONT, cruise_layer=2, customer_buildings=True)),
                         ("continuous, 60 m, customers in buildings, 32 drones",
                          dict(**CONT, **FLEET32, cruise_layer=2, customer_buildings=True))],
            "metrics": ["delivery_rate", "avg_delivery_time", "p95_delivery_time", "on_time_rate",
                        "energy_per_delivery", "upper_layer_share", "over_building_top_share", "collisions",
                        "wall_time_s"],
            "extra": ["delivered", "orders", "failed", "cells_flown", "hover_ticks", "swaps", "dead_drones",
                      "over_building_share", "climbs", "drop_height_mean_m", "rooftop_drop_share"],
        },
        "wind": {
            "question": "Continuous flight: how much wind can the energy-safe fleet fly in?",
            "note": "Reservations + ORCA, default configuration (12 drones). Mean wind at 30 m and RMS gusts: calm "
                    "0/0, moderate 5/1.5, strong 8/2.5, severe 10/3 m/s.",
            "variants": [(f"{w} wind", dict(**CONT, **wind(w))) for w in WIND_PRESETS],
            "metrics": ["delivery_rate", "avg_delivery_time", "energy_per_delivery", "flight_hours",
                        "separation_losses", "min_separation_m", "tracking_error_mean_m", "emergencies"],
            "extra": ["delivered", "orders", "failed", "swaps", "orca_per_drone_hour", "tracking_error_p95_m",
                      "on_time_rate", "building_intrusions"],
        },
    }


def run_one(job: tuple[dict, int]) -> dict:
    overrides, seed = job
    return Simulation(dataclasses.replace(BASE, seed=seed, **overrides)).run()


def run_variant(overrides: dict, seeds: list[int], pool=None) -> list[dict]:
    jobs = [(overrides, s) for s in seeds]
    return list(pool.map(run_one, jobs)) if pool else [run_one(j) for j in jobs]


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
    "over_building_top_share": "over roofs at 90 m",
    "separation_losses": "LoS events", "separation_loss_s": "LoS pair-s", "min_separation_m": "min sep (m)",
    "orca_per_drone_hour": "ORCA / drone-h", "tracking_error_mean_m": "track err (m)",
    "flight_hours": "flight h",
}
PERCENT = {"delivery_rate", "on_time_rate", "utilisation", "upper_layer_share", "over_building_top_share"}


def fmt(k: str, s: dict) -> str:
    m, sd = s["mean"], s["std"]
    if k in PERCENT:
        return f"{100 * m:.1f}% ±{100 * sd:.1f}"
    if abs(m) >= 100:
        return f"{m:.0f} ±{sd:.0f}"
    return f"{m:.2f} ±{sd:.2f}" if abs(m) < 10 else f"{m:.1f} ±{sd:.1f}"


def to_markdown(name: str, exp: dict, rows: list[tuple[str, dict]], n_seeds: int) -> str:
    keys = exp["metrics"]
    note = f" {exp['note']}" if exp.get("note") else ""
    lines = [f"### {name.capitalize()}: {exp['question']}", "",
             f"Mean ± standard deviation over {n_seeds} seeds.{note}", "",
             "| configuration | " + " | ".join(LABELS.get(k, k) for k in keys) + " |",
             "|---|" + "---:|" * len(keys)]
    for label, summ in rows:
        lines.append(f"| {label} | " + " | ".join(fmt(k, summ[k]) for k in keys) + " |")
    return "\n".join(lines) + "\n"


def header(seeds: list[int]) -> list[str]:
    return ["# Experiment results", "",
            f"Seeds {seeds[0]}..{seeds[-1]}; default scenario: {BASE.width}x{BASE.height} city, "
            f"{BASE.n_hubs} hubs, {BASE.n_stations} swap stations, {BASE.n_drones} drones, "
            f"{BASE.order_rate} orders/tick for {BASE.order_until} ticks, gust p={BASE.gust_prob}, "
            f"{BASE.n_layers} flight layer (the layer experiment varies it; the continuous-flight sections, "
            f"tactical, motion and wind, use the default 3 layers). Times are in ticks (1 tick ~ 10 s).", ""]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--only", nargs="*", help="run only these experiments (the others keep their results)")
    ap.add_argument("--out", default="results")
    ap.add_argument("--no-charts", action="store_true")
    ap.add_argument("--jobs", type=int, default=1,
                    help="parallel worker processes (results are identical; timings are per process)")
    a = ap.parse_args(argv)

    seeds = list(range(1, a.seeds + 1))
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    catalogue = experiments()
    unknown = set(a.only or ()) - set(catalogue)
    if unknown:
        ap.error(f"unknown experiment(s): {', '.join(sorted(unknown))}")

    # keep the sections that are not re-run (and their compute time)
    all_results: dict = {}
    earlier_seconds = 0.0
    jpath, mpath = out / "experiments.json", out / "experiments.md"
    if a.only and jpath.exists():
        all_results = {k: v for k, v in json.loads(jpath.read_text()).items() if k in catalogue}
        untimed = [k for k, v in all_results.items() if "seconds" not in v]
        m = re.search(r"_Total compute: ([\d.]+) s", mpath.read_text()) if mpath.exists() else None
        if untimed and m and not set(untimed) & set(a.only):
            earlier_seconds = float(m.group(1)) - sum(v.get("seconds", 0.0) for v in all_results.values())

    pool = ProcessPoolExecutor(a.jobs) if a.jobs > 1 else None
    try:
        for name, exp in catalogue.items():
            if a.only and name not in a.only:
                continue
            print(f"== {name}: {exp['question']}", flush=True)
            t_exp = time.perf_counter()
            res = {"question": exp["question"], "variants": []}
            keys = sorted(set(exp["metrics"]) | set(exp.get("extra", ())) |
                          {"collisions", "delivery_rate", "avg_delivery_time", "dead_drones"})
            for label, ov in exp["variants"]:
                runs = run_variant(ov, seeds, pool)
                summ = summarise(runs, keys)
                res["variants"].append({"label": label, "overrides": ov, "summary": summ, "runs": runs})
                print(f"   {label:30s} " + "  ".join(f"{LABELS.get(k, k)}={fmt(k, summ[k])}"
                                                      for k in exp["metrics"][:5]), flush=True)
            res["seconds"] = time.perf_counter() - t_exp
            all_results[name] = res
    finally:
        if pool:
            pool.shutdown()

    md = header(seeds)
    for name, exp in catalogue.items():
        if name in all_results:
            res = all_results[name]
            rows = [(v["label"], v["summary"]) for v in res["variants"]]
            md.append(to_markdown(name, exp, rows, len(res["variants"][0]["runs"])))
    total = earlier_seconds + sum(v.get("seconds", 0.0) for v in all_results.values())
    md.append(f"_Total compute: {total:.1f} s._\n")
    mpath.write_text("\n".join(md))
    ordered = {name: all_results[name] for name in catalogue if name in all_results}
    jpath.write_text(json.dumps(ordered, indent=1))
    print(f"\nWrote {mpath} and {jpath}")
    if not a.no_charts:
        from dronefleet.charts import write_charts
        ran = {k: v for k, v in ordered.items() if not a.only or k in a.only}
        for p in write_charts(ran, out / "figures"):
            print(f"chart: {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
