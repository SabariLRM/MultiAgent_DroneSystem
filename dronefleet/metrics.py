"""Performance measures for one simulation run."""

from __future__ import annotations

import statistics


def _pct(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    k = min(len(s) - 1, max(0, round(q * (len(s) - 1))))
    return float(s[k])


def compute_metrics(sim) -> dict:
    orders = list(sim.dispatcher.orders.values())
    delivered = [o for o in orders if o.status == "delivered"]
    lead = [o.delivered_t - o.created_t for o in delivered]
    express = [o.delivered_t - o.created_t for o in delivered if o.express]
    on_time = sum(1 for o in delivered if o.delivered_t <= o.deadline_t)
    ds = [d.stats for d in sim.drones]
    tot = lambda k: sum(s[k] for s in ds)  # noqa: E731
    energy = tot("energy_used")
    ticks = max(1, sim.t)
    ps = sim.planner.stats
    vertex = sum(1 for c in sim.collisions if c[1] == "vertex")
    edge = sum(1 for c in sim.collisions if c[1] == "edge")
    m = {
        "ticks": sim.t,
        "orders": len(orders),
        "delivered": len(delivered),
        "failed": sum(1 for o in orders if o.status == "failed"),
        "undelivered": sum(1 for o in orders if o.status not in ("delivered", "failed")),
        "delivery_rate": len(delivered) / len(orders) if orders else 1.0,
        "avg_delivery_time": statistics.fmean(lead) if lead else 0.0,
        "p95_delivery_time": _pct(lead, 0.95),
        "avg_express_time": statistics.fmean(express) if express else 0.0,
        "on_time_rate": on_time / len(delivered) if delivered else 0.0,
        "throughput_per_100t": 100 * len(delivered) / ticks,
        "collisions": vertex + edge,
        "vertex_conflicts": vertex,
        "edge_conflicts": edge,
        "nfz_violations": sim.nfz_violations,
        "energy_total": energy,
        "energy_per_delivery": energy / len(delivered) if delivered else 0.0,
        "cells_flown": tot("cells_flown"),
        "hover_ticks": tot("hover_ticks"),
        "climbs": tot("climbs"),
        "descents": tot("descents"),
        "upper_layer_share": tot("upper_layer_ticks") / max(1, tot("airborne_ticks")),
        "swaps": sum(s.swaps_done for s in sim.stations),
        "avg_swap_wait": (sum(s.wait_ticks_total for s in sim.stations) / max(1, sum(s.swaps_done for s in sim.stations))),
        "max_station_queue": max((s.max_queue for s in sim.stations), default=0),
        "emergencies": tot("emergencies"),
        "dead_drones": sum(1 for d in sim.drones if not d.alive),
        "min_soc_seen": min((d.soc for d in sim.drones), default=0.0),
        "utilisation": tot("busy_ticks") / (ticks * len(sim.drones)),
        "replans": tot("replans"),
        "plan_repairs": tot("repairs"),
        "plan_failures": tot("plan_failures"),
        "deviations": tot("deviations"),
        "holds": tot("holds"),
        "yields": tot("yields"),
        "escalations": tot("escalations"),
        "forced_holds": sim.forced_holds,
        "gusts": sim.gusts,
        "planner_searches": ps.searches,
        "planner_expansions": ps.expansions,
        "planner_ms_per_search": 1000 * ps.seconds / max(1, ps.searches),
        "messages": sim.bus.total,
        "messages_excl_telemetry": sim.bus.total - sim.bus.topic_counts["telemetry"] - sim.bus.topic_counts["station_status"],
        "auction_rounds": sim.dispatcher.round_no,
        "failed_awards": sim.dispatcher.failed_awards,
        "wall_time_s": sim.wall_time,
    }
    flight = getattr(sim, "flight", None)
    if flight is not None:
        # continuous flight: physical collisions, separation, tracking and ORCA measures
        m.update(flight.metrics())
    return m


def format_metrics(m: dict) -> str:
    rows = [
        ("Orders delivered", f"{m['delivered']}/{m['orders']} ({m['delivery_rate']:.0%})"
                             f"  failed={m['failed']} open={m['undelivered']}"),
        ("Delivery time (ticks)", f"avg {m['avg_delivery_time']:.1f}  p95 {m['p95_delivery_time']:.0f}"
                                  f"  express avg {m['avg_express_time']:.1f}"),
        ("On-time rate", f"{m['on_time_rate']:.1%}"),
        ("Throughput", f"{m['throughput_per_100t']:.2f} deliveries / 100 ticks"),
        ("Collisions", f"{m['collisions']} (vertex {m['vertex_conflicts']}, head-on {m['edge_conflicts']})"
                       f"   NFZ violations {m['nfz_violations']}"),
        ("Energy", f"{m['energy_total']:.0f} total, {m['energy_per_delivery']:.1f} per delivery"),
        ("Altitude", f"{m['climbs']} climbs, {m['descents']} descents, "
                     f"{m['upper_layer_share']:.0%} of flight time above layer 1"),
        ("Battery", f"{m['swaps']} swaps, avg wait {m['avg_swap_wait']:.1f} ticks, "
                    f"{m['emergencies']} emergency diversions, {m['dead_drones']} drones lost"),
        ("Fleet utilisation", f"{m['utilisation']:.1%}"),
        ("Coordination", f"{m['replans']} plans, {m['plan_repairs']} repairs, {m['deviations']} deviations, "
                         f"{m['yields']} yields, {m['escalations']} escalations, {m['forced_holds']} reactive holds"),
        ("Planner", f"{m['planner_searches']} A* searches, {m['planner_expansions']} expansions, "
                    f"{m['planner_ms_per_search']:.2f} ms/search"),
        ("Messages", f"{m['messages']} total ({m['messages_excl_telemetry']} excluding telemetry), "
                     f"{m['auction_rounds']} auction rounds"),
        ("Sim", f"{m['ticks']} ticks in {m['wall_time_s']:.2f}s"),
    ]
    if "separation_losses" in m:
        rows[4:5] = [
            ("Collisions", f"{m['collisions']} (3-D distance under 2 x drone radius)   NFZ violations {m['nfz_violations']}"),
            ("Separation", f"{m['separation_losses']} losses ({m['separation_loss_s']:.0f} pair-seconds), "
                           f"minimum {m['min_separation_m']:.1f} m"),
            ("Tracking", f"error mean {m['tracking_error_mean_m']:.1f} m, p95 {m['tracking_error_p95_m']:.0f} m; "
                         f"{m['orca_interventions']} ORCA interventions ({m['orca_per_drone_hour']:.1f} per drone-hour), "
                         f"{m['stall_replans']} stall re-plans"),
        ]
    w = max(len(k) for k, _ in rows)
    return "\n".join(f"  {k:<{w}}  {v}" for k, v in rows)
