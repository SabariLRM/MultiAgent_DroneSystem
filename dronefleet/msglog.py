"""A human-readable record of every message the agents exchange.

The message bus calls :meth:`MessageLog.record` for each message it delivers
(when ``SimConfig.record_messages`` is on). Each message is turned into one
plain-English sentence at the moment it is sent, so it shows the state at
that time (a battery's charge, an estimated wait). After the run,
:meth:`MessageLog.write` produces a text file: a short summary, then every
message in order. Routine status reports (each drone's telemetry and each
station's status, every tick) are counted but only listed with ``everything``.
"""

from __future__ import annotations

from collections import Counter
from pathlib import Path

ACTS = {"cfp": "CALL FOR BIDS", "propose": "BID", "refuse": "REFUSE", "accept": "ACCEPT", "reject": "REJECT",
        "request": "REQUEST", "agree": "AGREE", "inform": "INFORM", "failure": "FAILURE", "cancel": "CANCEL"}
ROUTINE = ("telemetry", "station_status")
STATE_WORDS = {"idle": "free", "to_pickup": "flying to collect an order", "loading": "loading a parcel",
               "to_customer": "delivering", "returning": "returning to a hub", "to_station": "flying to a swap station",
               "queued": "waiting for a battery swap", "swapping": "having its battery swapped", "dead": "lost"}


KINDS = {("cfp", "cfp"): "auction announcements (dispatcher to all drones)",
         ("propose", "bid"): "bids (drone to dispatcher)", ("refuse", "bid"): "refusals to bid",
         ("accept", "award"): "auction wins (dispatcher to the winner)", ("reject", "award"): "auction losses",
         ("failure", "award"): "won orders handed back", ("request", "assign"): "direct assignments",
         ("agree", "assign"): "assignments accepted", ("refuse", "assign"): "assignments refused",
         ("failure", "task"): "orders handed back", ("inform", "picked_up"): "pick-up reports",
         ("inform", "delivered"): "delivery reports", ("inform", "package_lost"): "lost parcels",
         ("inform", "task_dropped"): "lost drones' orders", ("inform", "telemetry"): "drone status reports (every tick)",
         ("request", "reserve_swap"): "swap reservations (drone to station)", ("agree", "swap_reserved"): "swap reservations confirmed",
         ("cancel", "reserve_swap"): "swap reservations cancelled", ("request", "swap"): "swap requests on landing",
         ("inform", "swap_started"): "swaps started", ("inform", "swap_done"): "swaps done",
         ("inform", "station_status"): "station status broadcasts (every tick)",
         ("request", "yield"): "requests to give way (drone to drone)", ("inform", "nfz"): "no-fly-zone notices (air traffic control)"}


def agent_name(name: str) -> str:
    """dispatcher -> Dispatcher, drone3 -> Drone 3, station1 -> Station 1, drones -> all drones."""
    if name.startswith("drone") and name[5:].isdigit():
        return f"Drone {name[5:]}"
    if name.startswith("station") and name[7:].isdigit():
        return f"Station {name[7:]}"
    return {"dispatcher": "Dispatcher", "atc": "Air traffic control", "drones": "all drones",
            "stations": "all stations", "*": "everyone"}.get(name, name)


class MessageLog:
    def __init__(self, world):
        self.places = {tuple(h): f"Hub {i}" for i, h in enumerate(world.hubs)}
        self.places.update({tuple(s): f"Station {i}" for i, s in enumerate(world.stations)})
        self.n_layers = world.n_layers
        self.records: list[tuple[int, str, str, str, str, str]] = []   # t, from, to, act, topic, text
        self.counts: Counter = Counter()

    # ---------------------------------------------------------------- record
    def record(self, msg) -> None:
        topic = msg.content.get("type", msg.performative.value)
        act = msg.performative.value
        self.counts[(act, topic)] += 1
        self.records.append((msg.t, agent_name(msg.sender), agent_name(msg.receiver), act, topic, self.describe(msg)))

    def _order(self, o: dict, short: bool = False) -> str:
        hub = self.places.get(tuple(o["hub"]), f"the hub at {tuple(o['hub'])}")
        express = " (EXPRESS)" if o.get("express") else ""
        x, y = o["dest"]
        if short:
            return f"#{o['oid']}{express} {o['weight']:.1f} kg {hub} -> ({x}, {y}) due t={o['deadline_t']}"
        return (f"order #{o['oid']}{express}: {o['weight']:.1f} kg from {hub} to the customer at ({x}, {y}), "
                f"due by t={o['deadline_t']}")

    def describe(self, msg) -> str:
        c, p = msg.content, msg.performative.value
        kind = c.get("type", p)
        rnd = msg.conversation.removeprefix("cfp-") if msg.conversation.startswith("cfp-") else ""
        if kind == "cfp":
            orders = c["orders"]
            return (f"Auction round {rnd}: who can deliver {'this order' if len(orders) == 1 else 'these orders'}? "
                    + "; ".join(self._order(o, short=True) for o in orders))
        if kind == "bid" and p == "propose":
            parts = []
            for b in c["bids"]:
                via = f", after a battery swap at Station {b['via']}" if b.get("via") is not None else ""
                parts.append(f"order #{b['oid']} for cost {b['cost']:.1f}, delivered by t={b['eta']}{via}")
            return f"My bids in round {rnd}: " + "; ".join(parts)
        if kind == "bid":
            return f"I cannot take any of the orders in round {rnd}."
        if kind == "award" and p == "accept":
            return f"You win {self._order(c['order'])}."
        if kind == "award" and p == "reject":
            return f"You did not win anything in round {rnd}."
        if kind == "award":
            return f"I cannot take order #{c['oid']} after all ({c.get('reason', 'no reason given')})."
        if kind == "assign" and p == "request":
            return f"Please deliver {self._order(c['order'])}."
        if kind == "assign":
            return f"I {'accept' if p == 'agree' else 'refuse'} order #{c['oid']}."
        if kind == "task":
            return f"I am giving order #{c['oid']} back ({c.get('reason', 'no reason given')})."
        if kind == "picked_up":
            return f"I have picked up order #{c['oid']}."
        if kind == "delivered":
            return f"I have delivered order #{c['oid']}."
        if kind == "package_lost":
            return f"I have run out of battery in flight; order #{c['oid']} is lost with me."
        if kind == "task_dropped":
            return f"I was lost before collecting order #{c['oid']}."
        if kind == "telemetry":
            x, y = c["pos"]
            where = self.places.get((x, y), f"({x}, {y})")
            alt = "on the ground" if not c.get("alt") else f"on layer {c['alt']}"
            task = f", working on order #{c['task']}" if c.get("task") is not None else ""
            return f"Status: {STATE_WORDS.get(c['state'], c['state'])} at {where}, {alt}, battery {c['soc']:.0%}{task}."
        if kind == "reserve_swap" and p == "request":
            return f"Please reserve a battery swap for me; I will arrive around t={c['eta']}."
        if kind == "reserve_swap":
            return "I no longer need my swap reservation."
        if kind == "swap_reserved":
            w = c.get("est_wait", 0)
            return f"Reserved. Expected wait when you arrive: {w} tick{'s' if w != 1 else ''}."
        if kind == "swap":
            return f"I have landed here; please swap my battery ({c['pack'].soc:.0%} left)."
        if kind == "swap_started":
            return "Your battery swap has started."
        if kind == "swap_done":
            return f"Swap done: here is a fresh battery pack ({c['pack'].soc:.0%})."
        if kind == "station_status":
            w = c.get("est_wait_now", 0)
            return (f"Status: {c['queue']} waiting, {c['active']} being swapped, {c['charged']} charged spare "
                    f"pack{'s' if c['charged'] != 1 else ''}, {c.get('expected', 0)} on the way, expected wait "
                    f"{w} tick{'s' if w != 1 else ''}.")
        if kind == "yield":
            return (f"Please re-plan your route: Drone {c['by']} has claimed airspace you had booked "
                    f"(it was blocked and has priority).")
        if kind == "nfz":
            x0, y0, x1, y1 = c["rect"]
            z0, z1 = c.get("layers", (1, self.n_layers))
            layers = "" if (z0, z1) == (1, self.n_layers) else f", layers {z0}-{z1} only"
            return (f"No-fly zone over cells ({x0}, {y0}) to ({x1}, {y1}){layers}, closed from t={c['start_t']} "
                    f"to t={c['end_t']}. Re-plan if your route crosses it.")
        rest = ", ".join(f"{k}={v}" for k, v in c.items() if k != "type")
        return f"{kind}: {rest}" if rest else kind

    # ----------------------------------------------------------------- output
    def listed(self, everything: bool = False):
        return [r for r in self.records if everything or r[4] not in ROUTINE]

    def write(self, path, title: str, everything: bool = False) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        routine = {k: sum(n for (a, t), n in self.counts.items() if t == k) for k in ROUTINE}
        rows = self.listed(everything)
        lines = [title, "=" * len(title), "",
                 f"{len(self.records):,} messages in total. " +
                 ("All of them are listed below." if everything else
                  f"Listed below: the {len(rows):,} that are not routine status reports. Not listed one by one: "
                  f"{routine['telemetry']:,} drone status reports to the dispatcher (one per drone and tick) and "
                  f"{routine['station_status']:,} station status broadcasts (one per station and tick); "
                  f"run with --messages-all to include them."),
                 "", "Messages by kind", ""]
        label = lambda a, t: KINDS.get((a, t), f"{ACTS.get(a, a)} {t}")  # noqa: E731
        w = max(len(label(a, t)) for a, t in self.counts) + 2
        for (a, t), n in sorted(self.counts.items(), key=lambda kv: -kv[1]):
            lines.append(f"  {label(a, t).ljust(w)}{n:>8,}")
        lines += ["", "Every message", "",
                  f"{'tick':>5}  {'from':<20} {'to':<20} {'act':<14} message",
                  f"{'-' * 5}  {'-' * 20} {'-' * 20} {'-' * 14} {'-' * 40}"]
        for t, s, r, a, _, text in rows:
            lines.append(f"{t:>5}  {s:<20} {r:<20} {ACTS.get(a, a):<14} {text}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path
