"""Agent communication: FIPA-ACL style performatives over an in-process bus.

Agents never call each other's methods; they only exchange ``Message`` s. The
bus delivers into the recipient's inbox immediately, and the recipient reads
its inbox the next time it runs its ``step``. Because the simulation steps the
dispatcher, then the stations, then the drones, a round trip such as
CFP -> PROPOSE -> ACCEPT takes one to two ticks, like a real radio network with
latency.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
from enum import Enum
from itertools import count
from typing import Any


class Performative(str, Enum):
    CFP = "cfp"                      # call for proposals (task announcement)
    PROPOSE = "propose"              # bid
    REFUSE = "refuse"                # decline to bid / decline a request
    ACCEPT_PROPOSAL = "accept"       # award
    REJECT_PROPOSAL = "reject"       # lost the auction
    REQUEST = "request"              # ask someone to do something
    AGREE = "agree"                  # commit to a request
    INFORM = "inform"                # share a belief / report an event
    FAILURE = "failure"              # a commitment could not be honoured
    CANCEL = "cancel"                # withdraw a request


BROADCAST = "*"


@dataclass
class Message:
    sender: str
    receiver: str                    # agent name, a role ("drones") or "*"
    performative: Performative
    content: dict[str, Any]
    t: int
    conversation: str = ""
    mid: int = field(default=0)


class MessageBus:
    def __init__(self):
        self._agents: dict[str, "object"] = {}
        self._roles: dict[str, list[str]] = {}
        self._ids = count(1)
        self.counts: Counter = Counter()        # by performative
        self.topic_counts: Counter = Counter()  # by content["type"]
        self.log: deque = deque(maxlen=400)     # recent non-telemetry traffic

    def register(self, name: str, role: str, agent) -> None:
        self._agents[name] = agent
        self._roles.setdefault(role, []).append(name)

    def names(self, role: str) -> list[str]:
        return list(self._roles.get(role, []))

    def send(self, msg: Message) -> None:
        msg.mid = next(self._ids)
        if msg.receiver == BROADCAST:
            targets = [n for n in self._agents if n != msg.sender]
        elif msg.receiver in self._roles:
            targets = [n for n in self._roles[msg.receiver] if n != msg.sender]
        else:
            targets = [msg.receiver]
        for name in targets:
            agent = self._agents.get(name)
            if agent is not None:
                agent.inbox.append(msg)
        self.counts[msg.performative.value] += 1
        topic = msg.content.get("type", msg.performative.value)
        self.topic_counts[topic] += 1
        if topic not in ("telemetry", "station_status"):
            self.log.append(msg)

    @property
    def total(self) -> int:
        return sum(self.counts.values())
