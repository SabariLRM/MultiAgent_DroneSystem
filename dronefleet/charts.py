"""Dependency-free SVG charts for the experiment report.

Small multiples of column charts (one panel per metric, never two y-scales on
one plot) and line charts. Colours come from a validated categorical palette
(first three slots, which stay distinguishable for colour-blind readers), or,
for an ordered quantity such as the number of flight layers, a validated
one-hue ordinal ramp; text always uses ink tokens, never the series colour. Each SVG carries its own
light/dark styles and native ``<title>`` tooltips on every mark, and the
Markdown report prints the same numbers as a table.
"""

from __future__ import annotations

import math
from html import escape
from pathlib import Path

SERIES = ["s1", "s2", "s3"]
ORDINAL = ["o1", "o2", "o3"]      # blue ramp, light -> dark (dark -> light on the dark surface)
STYLE = """
<style>
  svg { --surface:#fcfcfb; --ink:#0b0b0b; --ink2:#52514e; --muted:#898781; --grid:#e1e0d9;
        --base:#c3c2b7; --s1:#2a78d6; --s2:#eb6834; --s3:#1baf7a; --o1:#86b6ef; --o2:#2a78d6; --o3:#104281; }
  @media (prefers-color-scheme: dark) {
    svg { --surface:#1a1a19; --ink:#ffffff; --ink2:#c3c2b7; --muted:#898781; --grid:#2c2c2a;
          --base:#383835; --s1:#3987e5; --s2:#d95926; --s3:#199e70; --o1:#184f95; --o2:#3987e5; --o3:#9ec5f4; }
  }
  .bg { fill: var(--surface); }
  text { font-family: system-ui, -apple-system, "Segoe UI", sans-serif; fill: var(--ink2); font-size: 12px; }
  .title { fill: var(--ink); font-size: 15px; font-weight: 600; }
  .sub { fill: var(--ink2); font-size: 12px; }
  .ptitle { fill: var(--ink); font-size: 13px; font-weight: 600; }
  .tick { fill: var(--muted); font-size: 11px; font-variant-numeric: tabular-nums; }
  .val { fill: var(--ink2); font-size: 11px; font-variant-numeric: tabular-nums; }
  .grid { stroke: var(--grid); stroke-width: 1; }
  .base { stroke: var(--base); stroke-width: 1; }
  .err { stroke: var(--muted); stroke-width: 1; }
  .s1 { fill: var(--s1); } .s2 { fill: var(--s2); } .s3 { fill: var(--s3); }
  .o1 { fill: var(--o1); } .o2 { fill: var(--o2); } .o3 { fill: var(--o3); }
  .l1 { stroke: var(--s1); } .l2 { stroke: var(--s2); } .l3 { stroke: var(--s3); }
  .line { fill: none; stroke-width: 2; stroke-linejoin: round; stroke-linecap: round; }
  .dot { stroke: var(--surface); stroke-width: 2; }
</style>"""


def _nice_max(v: float) -> tuple[float, float]:
    """Round an axis maximum up to a clean number; return (max, step)."""
    if v <= 0:
        return 1.0, 0.25
    raw = v / 4
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if step >= raw:
            break
    return step * math.ceil(v / step), step


def _fmt(v: float, pct: bool = False) -> str:
    if pct:
        return f"{100 * v:.0f}%"
    if v == 0:
        return "0"
    if abs(v) >= 1000:
        return f"{v / 1000:.1f}k"
    if abs(v) >= 100:
        return f"{v:.0f}"
    if abs(v) >= 10:
        return f"{v:.1f}".rstrip("0").rstrip(".")
    return f"{v:.2f}".rstrip("0").rstrip(".")


def _col(x: float, y: float, w: float, h: float, r: float = 4) -> str:
    r = min(r, w / 2, h)
    return (f"M{x:.1f},{y + h:.1f} L{x:.1f},{y + r:.1f} Q{x:.1f},{y:.1f} {x + r:.1f},{y:.1f} "
            f"L{x + w - r:.1f},{y:.1f} Q{x + w:.1f},{y:.1f} {x + w:.1f},{y + r:.1f} L{x + w:.1f},{y + h:.1f} Z")


def _legend(series: list[str], x: float, y: float, classes: list[str] = SERIES) -> str:
    out = []
    for i, name in enumerate(series):
        out.append(f'<rect x="{x:.0f}" y="{y - 9:.0f}" width="10" height="10" rx="2" class="{classes[i]}"/>'
                   f'<text x="{x + 15:.0f}" y="{y:.0f}">{escape(name)}</text>')
        x += 34 + 6.6 * len(name)
    return "".join(out)


def grouped_columns(title: str, subtitle: str, panels: list[dict], groups: list[str], series: list[str],
                    classes: list[str] = SERIES, values: bool = True, pw: int = 300) -> str:
    """panels: [{title, pct, values: {(group, series): (mean, std)}}]

    ``classes`` picks the colour slots (categorical by default, ``ORDINAL`` for
    ordered series); ``values=False`` drops the per-bar value labels when bars
    are too narrow for them (the tooltips and the report's table carry the numbers);
    ``pw`` is the width of one panel.
    """
    ph = 190
    top, left, gap = 108, 44, 26
    W = left + len(panels) * (pw + gap)
    H = top + ph + 52
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="100%" style="max-width:{W}px" '
             f'role="img" aria-label="{escape(title)}">', STYLE,
             f'<rect class="bg" width="{W}" height="{H}"/>',
             f'<text class="title" x="16" y="26">{escape(title)}</text>',
             f'<text class="sub" x="16" y="45">{escape(subtitle)}</text>',
             _legend(series, 16, 68, classes)]
    for pi, p in enumerate(panels):
        x0 = left + pi * (pw + gap)
        vmax = max((m + s) for m, s in p["values"].values()) if p["values"] else 1
        if p.get("pct"):
            vmax = min(1.0, vmax) if vmax > 0 else 1.0
        amax, step = _nice_max(vmax * 1.08)
        if p.get("pct"):
            amax, step = (1.0, 0.25) if vmax > 0.5 else _nice_max(vmax * 1.08)
        sy = ph / amax
        parts.append(f'<text class="ptitle" x="{x0}" y="{top - 14}">{escape(p["title"])}</text>')
        v = 0.0
        while v <= amax + 1e-9:
            y = top + ph - v * sy
            parts.append(f'<line class="grid" x1="{x0}" x2="{x0 + pw}" y1="{y:.1f}" y2="{y:.1f}"/>'
                         f'<text class="tick" x="{x0 - 6}" y="{y + 4:.1f}" text-anchor="end">{_fmt(v, p.get("pct"))}</text>')
            v += step
        base = top + ph
        parts.append(f'<line class="base" x1="{x0}" x2="{x0 + pw}" y1="{base}" y2="{base}"/>')
        band = pw / len(groups)
        bw = min(24, (band - 24) / len(series) - 2)
        for gi, g in enumerate(groups):
            gx = x0 + gi * band + (band - (bw + 2) * len(series) + 2) / 2
            for si, s in enumerate(series):
                if (g, s) not in p["values"]:
                    continue
                m, sd = p["values"][(g, s)]
                bx = gx + si * (bw + 2)
                h = max(0.0, m * sy)
                tip = f"{s} · {g}: {_fmt(m, p.get('pct'))} (±{_fmt(sd, p.get('pct'))})"
                if h > 0.5:
                    parts.append(f'<path class="{classes[si]}" d="{_col(bx, base - h, bw, h)}"><title>{escape(tip)}</title></path>')
                else:
                    parts.append(f'<rect class="{classes[si]}" x="{bx:.1f}" y="{base - 1:.1f}" width="{bw:.1f}" height="1"><title>{escape(tip)}</title></rect>')
                if sd > 0 and h > 0.5:
                    y1, y2 = base - min(amax, m + sd) * sy, base - max(0, m - sd) * sy
                    cx = bx + bw / 2
                    parts.append(f'<line class="err" x1="{cx:.1f}" x2="{cx:.1f}" y1="{y1:.1f}" y2="{y2:.1f}"/>')
                if values:
                    label_y = base - max(h, (min(amax, m + sd) * sy) if sd > 0 and h > 0.5 else h) - 5
                    parts.append(f'<text class="val" x="{bx + bw / 2:.1f}" y="{label_y:.1f}" text-anchor="middle">{_fmt(m, p.get("pct"))}</text>')
            parts.append(f'<text class="tick" x="{x0 + gi * band + band / 2:.1f}" y="{base + 18}" text-anchor="middle">{escape(g)}</text>')
    note = "Bars: mean over seeds; whiskers: ±1 standard deviation."
    if not values:
        note += " Hover a bar for its value; the report's table lists every number."
    parts.append(f'<text class="tick" x="16" y="{H - 10}">{note}</text>')
    parts.append("</svg>")
    return "\n".join(parts)


def line_panels(title: str, subtitle: str, xs: list[float], xlabel: str, panels: list[dict]) -> str:
    """panels: [{title, series: {name: [(mean, std), ...]}, pct, ymax}] - one y-scale per panel
    (``ymax`` optionally fixes the top of the axis, e.g. 1.0 for a share)."""
    pw, ph = 300, 180
    top, left, gap = 108, 44, 30
    W = left + len(panels) * (pw + gap)
    H = top + ph + 58
    names = []
    for p in panels:
        for n in p["series"]:
            if n not in names:
                names.append(n)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="100%" style="max-width:{W}px" '
             f'role="img" aria-label="{escape(title)}">', STYLE,
             f'<rect class="bg" width="{W}" height="{H}"/>',
             f'<text class="title" x="16" y="26">{escape(title)}</text>',
             f'<text class="sub" x="16" y="45">{escape(subtitle)}</text>']
    if len(names) > 1:
        parts.append(_legend(names, 16, 68))
    xmin, xmax = min(xs), max(xs)
    for pi, p in enumerate(panels):
        x0 = left + pi * (pw + gap)
        vmax = max(m + s for vals in p["series"].values() for m, s in vals)
        amax, step = (p["ymax"], p["ymax"] / 4) if p.get("ymax") else _nice_max(max(vmax, 1e-9) * 1.08)
        sy = ph / amax
        sx = lambda x: x0 + 10 + (x - xmin) / ((xmax - xmin) or 1) * (pw - 20)  # noqa: E731
        parts.append(f'<text class="ptitle" x="{x0}" y="{top - 14}">{escape(p["title"])}</text>')
        v = 0.0
        while v <= amax + 1e-9:
            y = top + ph - v * sy
            parts.append(f'<line class="grid" x1="{x0}" x2="{x0 + pw}" y1="{y:.1f}" y2="{y:.1f}"/>'
                         f'<text class="tick" x="{x0 - 6}" y="{y + 4:.1f}" text-anchor="end">{_fmt(v, p.get("pct"))}</text>')
            v += step
        base = top + ph
        parts.append(f'<line class="base" x1="{x0}" x2="{x0 + pw}" y1="{base}" y2="{base}"/>')
        for x in xs:
            parts.append(f'<text class="tick" x="{sx(x):.1f}" y="{base + 18}" text-anchor="middle">{_fmt(x)}</text>')
        parts.append(f'<text class="tick" x="{x0 + pw / 2:.1f}" y="{base + 36}" text-anchor="middle">{escape(xlabel)}</text>')
        for name, vals in p["series"].items():
            si = names.index(name)
            pts = [(sx(x), base - m * sy) for x, (m, _) in zip(xs, vals)]
            d = " ".join(f"{'M' if i == 0 else 'L'}{px:.1f},{py:.1f}" for i, (px, py) in enumerate(pts))
            parts.append(f'<path class="line l{si + 1}" d="{d}"/>')
            for (px, py), x, (m, sd) in zip(pts, xs, vals):
                tip = f"{name} @ {_fmt(x)}: {_fmt(m, p.get('pct'))} (±{_fmt(sd, p.get('pct'))})"
                parts.append(f'<circle class="dot {SERIES[si]}" cx="{px:.1f}" cy="{py:.1f}" r="4"><title>{escape(tip)}</title></circle>')
            ex, ey = pts[-1]
            parts.append(f'<text class="val" x="{ex - 6:.1f}" y="{ey - 9:.1f}" text-anchor="end">{_fmt(vals[-1][0], p.get("pct"))}</text>')
    parts.append("</svg>")
    return "\n".join(parts)


# ---------------------------------------------------------------------------
def _get(res: dict, exp: str, label: str, key: str) -> tuple[float, float]:
    for v in res[exp]["variants"]:
        if v["label"] == label:
            s = v["summary"][key]
            return s["mean"], s["std"]
    raise KeyError(label)


def write_charts(results: dict, outdir: str | Path) -> list[Path]:
    out = Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    written = []
    groups = ["12 drones", "24 drones"]

    if "coordination" in results:
        strat = {"cooperative": "cooperative", "reactive only": "reactive only", "none": "none"}
        lab = lambda s, g: f"{s} ({g})"  # noqa: E731
        panels = []
        for key, title, pct in [("collisions", "Collisions per run", False),
                                ("avg_delivery_time", "Average delivery time (ticks)", False),
                                ("dead_drones", "Drones lost (battery depleted)", False)]:
            panels.append({"title": title, "pct": pct, "values": {
                (g, s): _get(results, "coordination", lab(s, g), key) for g in groups for s in strat}})
        svg = grouped_columns("Collision avoidance strategies",
                              "Only reservation-based cooperative planning is both collision-free and fast.",
                              panels, groups, list(strat))
        written.append(_write(out / "coordination.svg", svg))

    if "allocation" in results:
        strat = ["contract net", "nearest idle", "round robin"]
        panels = []
        for key, title, pct in [("avg_delivery_time", "Average delivery time (ticks)", False),
                                ("p95_delivery_time", "95th percentile delivery time", False),
                                ("energy_per_delivery", "Energy per delivery", False)]:
            panels.append({"title": title, "pct": pct, "values": {
                (g, s): _get(results, "allocation", f"{s} ({g})", key) for g in groups for s in strat}})
        svg = grouped_columns("Task allocation strategies",
                              "Contract-net auctions use each drone's private state (battery, ETA, swap plans).",
                              panels, groups, strat)
        written.append(_write(out / "allocation.svg", svg))

    if "battery" in results:
        labels = {("12 drones", "predictive"): "predictive (energy-aware)",
                  ("12 drones", "naive 30%"): "naive 30% threshold",
                  ("24 drones", "predictive"): "predictive, 24 drones",
                  ("24 drones", "naive 30%"): "naive, 24 drones"}
        panels = []
        for key, title, pct in [("dead_drones", "Drones lost in flight", False),
                                ("failed", "Orders lost", False),
                                ("delivery_rate", "Orders delivered", True)]:
            panels.append({"title": title, "pct": pct, "values": {
                k: _get(results, "battery", v, key) for k, v in labels.items()}})
        svg = grouped_columns("Battery management policies",
                              "Checking every plan against the energy budget prevents in-flight depletion.",
                              panels, groups, ["predictive", "naive 30%"])
        written.append(_write(out / "battery.svg", svg))

    if "scalability" in results:
        vs = results["scalability"]["variants"]
        xs = [v["overrides"]["n_drones"] for v in vs]
        pan = lambda key, title, pct=False: {  # noqa: E731
            "title": title, "pct": pct,
            "series": {"fleet": [(v["summary"][key]["mean"], v["summary"][key]["std"]) for v in vs]}}
        svg = line_panels("Scalability with fleet size",
                          "Demand grows with the fleet. Throughput keeps rising, but past 16 drones the three "
                          "swap stations saturate and delivery times grow.",
                          xs, "drones in fleet",
                          [pan("throughput_per_100t", "Deliveries per 100 ticks"),
                           pan("avg_delivery_time", "Average delivery time (ticks)"),
                           pan("avg_swap_wait", "Average wait at a swap station (ticks)"),
                           pan("planner_ms_per_search", "Planner time per A* search (ms)")])
        written.append(_write(out / "scalability.svg", svg))

    if "infrastructure" in results:
        groups_i = ["1 bay", "2 bays"]
        packs = ["3 spare packs", "6 spare packs"]
        lab = lambda g, s: f"{g}, {s}"  # noqa: E731
        panels = []
        for key, title, pct in [("avg_swap_wait", "Average swap wait (ticks)", False),
                                ("avg_delivery_time", "Average delivery time (ticks)", False),
                                ("on_time_rate", "On-time deliveries", True)]:
            panels.append({"title": title, "pct": pct, "values": {
                (g, s): _get(results, "infrastructure", lab(g, s), key) for g in groups_i for s in packs}})
        svg = grouped_columns("Battery-swap infrastructure (24 drones)",
                              "Spare packs, not swap bays, are the binding constraint: chargers need time.",
                              panels, groups_i, packs)
        written.append(_write(out / "infrastructure.svg", svg))

    if "robustness" in results:
        vs = results["robustness"]["variants"]
        xs = [v["overrides"]["gust_prob"] for v in vs]
        ser = lambda key: [(v["summary"][key]["mean"], v["summary"][key]["std"]) for v in vs]  # noqa: E731
        svg = line_panels("Robustness to wind gusts: execution events",
                          "Gusts knock drones off-plan; cheap plan repair absorbs most deviations, collisions stay at zero.",
                          xs, "gust probability per move",
                          [{"title": "Events per run", "series": {"deviations": ser("deviations"),
                                                                  "plan repairs": ser("plan_repairs"),
                                                                  "collisions": ser("collisions")}}])
        written.append(_write(out / "robustness.svg", svg))
        svg = line_panels("Robustness to wind gusts: service level",
                          "Service degrades gracefully as execution noise grows.",
                          xs, "gust probability per move",
                          [{"title": "Average delivery time (ticks)", "series": {"fleet": ser("avg_delivery_time")}},
                           {"title": "On-time rate", "pct": True, "series": {"fleet": ser("on_time_rate")}}])
        written.append(_write(out / "robustness_service.svg", svg))

    if "layers" in results:
        layers = ["1 layer", "3 layers", "5 layers"]
        groups_l = [f"{n} {rule}" for n in (24, 32) for rule in ("free", "heading")]
        lab = lambda g, s: f"{s}, {g.split()[1]} ({g.split()[0]} drones)"  # noqa: E731
        panels = []
        for key, title in [("avg_delivery_time", "Average delivery time (ticks)"),
                           ("forced_holds", "Reactive holds per run"),
                           ("energy_per_delivery", "Energy per delivery"),
                           ("planner_ms_per_search", "Planner time per A* search (ms)")]:
            panels.append({"title": title, "pct": False, "values": {
                (g, s): _get(results, "layers", lab(g, s), key) for g in groups_l for s in layers}})
        svg = grouped_columns("Altitude layers (24 and 32 drones, 6 spare packs per station)",
                              "Groups: fleet size and layer rule (free, or east/west on odd layers and north/south "
                              "on even). With one layer the heading rule has no effect.",
                              panels, groups_l, layers, classes=ORDINAL, values=False)
        written.append(_write(out / "layers.svg", svg))

    if "tactical" in results:
        strategies = ["reservations only", "ORCA only", "reservations + ORCA"]
        winds = ["calm", "moderate", "strong"]
        groups_t = [f"{w} {n}" for n in (12, 24) for w in winds]
        lab = lambda g, s: f"{s}, {g.split()[0]} wind ({g.split()[1]} drones)"  # noqa: E731
        panels = []
        for key, title, pct in [("separation_losses", "Losses of separation per run", False),
                                ("min_separation_m", "Closest approach in a run (m)", False),
                                ("avg_delivery_time", "Average delivery time (ticks)", False)]:
            panels.append({"title": title, "pct": pct, "values": {
                (g, s): _get(results, "tactical", lab(g, s), key) for g in groups_t for s in strategies}})
        svg = grouped_columns("Continuous flight: strategic reservations vs tactical ORCA",
                              "Groups: wind (calm, moderate 5 m/s, strong 8 m/s) and fleet size. A loss of separation "
                              "is two drones closer than 40 m horizontally and 15 m vertically.",
                              panels, groups_t, strategies, values=False, pw=430)
        written.append(_write(out / "tactical.svg", svg))

    if "motion" in results:
        modes = ["grid (default)", "continuous, calm", "continuous, moderate wind (default)"]
        names = ["grid cells", "continuous, calm", "continuous, moderate wind"]
        panels = []
        for key, title, pct in [("avg_delivery_time", "Average delivery time (ticks)", False),
                                ("energy_per_delivery", "Energy per delivery", False),
                                ("swaps", "Battery swaps per run", False)]:
            panels.append({"title": title, "pct": pct, "values": {
                ("12 drones", n): _get(results, "motion", m, key) for m, n in zip(modes, names)}})
        svg = grouped_columns("Grid cells vs continuous flight (default configuration)",
                              "Same seeds, cities and orders; continuous flight adds real take-offs, landings, "
                              "acceleration, avoidance and wind.",
                              panels, ["12 drones"], names)
        written.append(_write(out / "motion.svg", svg))

    if "wind" in results:
        vs = results["wind"]["variants"]
        xs = [v["overrides"].get("wind_mean", 0.0) for v in vs]
        ser = lambda key: [(v["summary"][key]["mean"], v["summary"][key]["std"]) for v in vs]  # noqa: E731
        svg = line_panels("Continuous flight in wind (reservations + ORCA, 12 drones)",
                          "Headwinds cost energy and the airspeed limit slows drones; the energy check keeps every "
                          "drone safe by refusing what it cannot price within its battery.",
                          xs, "mean wind at 30 m (m/s)",
                          [{"title": "Orders delivered", "pct": True, "ymax": 1.0, "series": {"fleet": ser("delivery_rate")}},
                           {"title": "Energy per delivery", "series": {"fleet": ser("energy_per_delivery")}},
                           {"title": "Flight hours per run", "series": {"fleet": ser("flight_hours")}}])
        written.append(_write(out / "wind.svg", svg))
    return written


def _write(path: Path, svg: str) -> Path:
    path.write_text(svg, encoding="utf-8")
    return path
