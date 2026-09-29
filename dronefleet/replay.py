"""Records a compact trace of a run and exports a self-contained HTML replay."""

from __future__ import annotations

import json
from pathlib import Path

TEMPLATE = Path(__file__).with_name("viewer_template.html")
STATE_NAMES = ["idle", "to_pickup", "loading", "to_customer", "returning", "to_station",
               "queued", "swapping", "dead"]
STATE_IDX = {n: i for i, n in enumerate(STATE_NAMES)}
# why a drone did not follow its plan this tick (or what it is doing in place)
FLAG_NONE, FLAG_WIND, FLAG_GIVE_WAY, FLAG_HOLDING, FLAG_LOWERING = range(5)


class TraceRecorder:
    def __init__(self, sim):
        self.sim = sim
        self.frames: list[dict] = []
        self.plans: dict[int, list] = {d.did: [] for d in sim.drones}
        self._seen = {d.did: 0 for d in sim.drones}
        self.W = sim.world.width
        self.record(0, ["simulation start"])

    def _enc(self, cell) -> int:
        return cell[1] * self.W + cell[0]

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
            return d.pos
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
                self._enc(post) if post else -1, self._eta(d, t),
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
        return {
            "meta": {"seed": cfg.seed, "allocation": cfg.allocation, "coordination": cfg.coordination,
                     "battery_policy": cfg.battery_policy, "n_drones": cfg.n_drones,
                     "gust_prob": cfg.gust_prob, "capacity": cfg.battery_capacity},
            "world": {"w": w.width, "h": w.height, "blocked": sorted(self._enc(c) for c in w.blocked),
                      "hubs": w.hubs, "stations": w.stations, "customers": w.customers,
                      "nfz": [{"rect": z.rect, "announce": z.announce_t, "start": z.start_t, "end": z.end_t}
                              for z in w.nfzs]},
            "frames": self.frames,
            "plans": {str(k): v for k, v in self.plans.items()},
            "orders": orders,
            "metrics": metrics or {},
            "states": STATE_NAMES,
        }


def export_html(sim, metrics: dict, path: str | Path) -> Path:
    if sim.trace is None:
        raise ValueError("run the simulation with record_trace=True to export a replay")
    data = json.dumps(sim.trace.to_dict(metrics), separators=(",", ":"))
    html = TEMPLATE.read_text(encoding="utf-8").replace("__REPLAY_DATA__", data.replace("</", "<\\/"))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(html, encoding="utf-8")
    return path


def export_json(sim, metrics: dict, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(sim.trace.to_dict(metrics)), encoding="utf-8")
    return path
