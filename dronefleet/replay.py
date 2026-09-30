"""Records a compact trace of a run and exports self-contained HTML replays.

Two viewers share one core (``viewer_core.js`` / ``viewer_core.css``, inlined
at export time): the top-down 2-D map (``viewer_template.html``, works
offline) and a 3-D scene (``viewer3d_template.html``, loads three.js from a
CDN).

Cells are encoded as integers ``(z * H + y) * W + x``; sites such as pads,
customers and building footprints have ``z = 0``, so their code is the plain
2-D one.

Continuous-flight runs add a ``track``: every drone's position every
``trace_dt`` seconds (2 s), in whole metres, as one flat integer array per
drone (the first sample absolute, then differences, which keeps the numbers
short). Losses of separation, ORCA interventions and collisions are lists of
intervals or events, and the wind at the city centre is sampled with the
positions. The per-tick frames stay as they are (they drive the side panels),
so a grid replay simply has no track.
"""

from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).parent
TEMPLATES = {"2d": HERE / "viewer_template.html", "3d": HERE / "viewer3d_template.html"}
TEMPLATE = TEMPLATES["2d"]
CORE_JS = HERE / "viewer_core.js"
CORE_CSS = HERE / "viewer_core.css"
LAYER_METRES = 30                  # nominal height of one flight layer (see config.py)
STATE_NAMES = ["idle", "to_pickup", "loading", "to_customer", "returning", "to_station",
               "queued", "swapping", "dead"]
STATE_IDX = {n: i for i, n in enumerate(STATE_NAMES)}
# why a drone did not follow its plan this tick (or what it is doing in place)
FLAG_NONE, FLAG_WIND, FLAG_GIVE_WAY, FLAG_HOLDING, FLAG_LOWERING = range(5)


class TraceRecorder:
    """Per tick and per drone: ``[x, y, airborne, state, soc %, order, carrying, express,
    target, flag, trip reason, landing pad after the drop, eta, layer]``."""

    def __init__(self, sim):
        self.sim = sim
        self.frames: list[dict] = []
        self.plans: dict[int, list] = {d.did: [] for d in sim.drones}
        self._seen = {d.did: 0 for d in sim.drones}
        self.W = sim.world.width
        self.H = sim.world.height
        self.record(0, ["simulation start"])

    def _enc(self, cell) -> int:
        z = cell[2] if len(cell) > 2 else 0
        return (z * self.H + cell[1]) * self.W + cell[0]

    def _target(self, d):
        """Where the drone is heading right now (or where it is being served)."""
        st = d.state.value
        if st in ("to_pickup", "loading") and d.task:
            return d.task["hub"]
        if st == "to_customer" and d.task:
            return d.task["dest"]
        if st in ("returning", "to_station") and d.post_pad:
            return d.post_pad
        if st in ("queued", "swapping"):
            return d.site
        return None

    def _eta(self, d, t: int) -> int:
        """Tick at which the drone reaches its current target, from its committed plan."""
        if d.plan is None:
            return -1
        if d.state.value == "to_customer":
            done = next((s.t for s in d.plan.remaining(t) if s.tag == "drop_done"), None)
            if done is not None:
                return done
        return d.plan.end_t

    def _flag(self, d, t: int) -> int:
        if not d.alive:
            return FLAG_NONE
        if d.airborne and d._in_drop(t):
            return FLAG_LOWERING
        if d.did in self.sim.last_gusted:
            return FLAG_WIND
        if d.did in self.sim.last_forced:
            return FLAG_GIVE_WAY
        if d.airborne and d.plan is None:
            return FLAG_HOLDING
        return FLAG_NONE

    def record(self, t: int, events: list[str]) -> None:
        sim = self.sim
        drones = []
        for d in sim.drones:
            task = d.task
            tgt = self._target(d)
            post = d.post_pad if d.state.value == "to_customer" else None
            drones.append([
                d.pos[0], d.pos[1], int(d.airborne), STATE_IDX[d.state.value], round(d.soc * 100),
                task["oid"] if task else -1, int(d.carrying), int(bool(task and task["express"])),
                self._enc(tgt) if tgt else -1, self._flag(d, t), d.trip_reason,
                self._enc(post) if post else -1, self._eta(d, t), d.pos[2],
            ])
            log = d.plan_log
            for pt, cells in log[self._seen[d.did]:]:
                self.plans[d.did].append([pt, [self._enc(c) for c in cells]])
            self._seen[d.did] = len(log)
            # keep memory flat during long runs
            if len(log) > 64:
                del log[:-8]
                self._seen[d.did] = len(log)
        counts = [0, 0, 0, 0]
        for o in sim.dispatcher.orders.values():
            if o.status == "pending":
                counts[0] += 1
            elif o.status in ("assigned", "picked"):
                counts[1] += 1
            elif o.status == "delivered":
                counts[2] += 1
            else:
                counts[3] += 1
        self.frames.append({
            "t": t, "d": drones, "s": [s.snapshot() for s in sim.stations], "o": counts,
            "c": len(sim.collisions), "e": events[:14],
        })

    def to_dict(self, metrics: dict | None = None) -> dict:
        sim, w = self.sim, self.sim.world
        orders = [[o.oid, w.hubs.index(o.hub), o.dest[0], o.dest[1], o.created_t, o.deadline_t,
                   o.delivered_t if o.delivered_t is not None else -1, int(o.express),
                   o.drone if o.drone is not None else -1, o.status, o.weight]
                  for o in sorted(sim.dispatcher.orders.values(), key=lambda o: o.oid)]
        cfg = sim.cfg
        blocked = sorted(w.blocked, key=self._enc)
        meta = {"seed": cfg.seed, "allocation": cfg.allocation, "coordination": cfg.coordination,
                "battery_policy": cfg.battery_policy, "n_drones": cfg.n_drones,
                "gust_prob": cfg.gust_prob, "capacity": cfg.battery_capacity,
                "n_layers": w.n_layers, "layer_rule": w.layer_rule, "layer_m": LAYER_METRES}
        extra = {}
        if getattr(sim, "flight", None) is not None:
            meta.update(motion="continuous", tactical=cfg.tactical, tick_s=cfg.tick_s, cell_m=cfg.cell_m,
                        layer_m=cfg.layer_m, wind=[cfg.wind_mean, cfg.wind_dir_deg, cfg.wind_gust])
            extra["track"] = sim.flight.trace(sim.t * cfg.tick_s)
        return {
            "meta": meta,
            "world": {"w": w.width, "h": w.height, "layers": w.n_layers, "layer_rule": w.layer_rule,
                      "blocked": [self._enc(c) for c in blocked], "heights": [w.heights[c] for c in blocked],
                      "hubs": w.hubs, "stations": w.stations, "customers": w.customers,
                      "nfz": [{"rect": z.rect, "announce": z.announce_t, "start": z.start_t, "end": z.end_t,
                               "layers": list(z.layers(w.n_layers))}
                              for z in w.nfzs]},
            "frames": self.frames,
            "plans": {str(k): v for k, v in self.plans.items()},
            "orders": orders,
            "metrics": metrics or {},
            "states": STATE_NAMES,
            **extra,
        }


def render_html(data: dict, view: str = "2d") -> str:
    """Fill a viewer template with the shared core and the replay data."""
    if view not in TEMPLATES:
        raise ValueError(f"view must be one of {tuple(TEMPLATES)}")
    html = TEMPLATES[view].read_text(encoding="utf-8")
    html = html.replace("/*__VIEWER_CORE_CSS__*/", CORE_CSS.read_text(encoding="utf-8"))
    html = html.replace("/*__VIEWER_CORE_JS__*/", CORE_JS.read_text(encoding="utf-8"))
    payload = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    return html.replace("__REPLAY_DATA__", payload)


def export_html(sim, metrics: dict, path: str | Path, view: str = "2d") -> Path:
    """Write a self-contained replay; ``view`` is "2d" (offline) or "3d" (needs internet for three.js)."""
    if sim.trace is None:
        raise ValueError("run the simulation with record_trace=True to export a replay")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_html(sim.trace.to_dict(metrics), view), encoding="utf-8")
    return path


def export_json(sim, metrics: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(sim.trace.to_dict(metrics)), encoding="utf-8")
    return path
