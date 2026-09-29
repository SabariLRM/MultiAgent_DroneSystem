from __future__ import annotations

from collections import deque

from ..messages import Message, MessageBus, Performative


class Agent:
    """Minimal agent: a name, an inbox, and a way to talk."""

    role = "agent"

    def __init__(self, name: str, bus: MessageBus):
        self.name = name
        self.bus = bus
        self.inbox: deque[Message] = deque()
        bus.register(name, self.role, self)

    def send(self, receiver: str, perf: Performative, t: int, conversation: str = "", **content) -> None:
        self.bus.send(Message(self.name, receiver, perf, content, t, conversation))

    def drain(self) -> list[Message]:
        msgs = list(self.inbox)
        self.inbox.clear()
        return msgs
