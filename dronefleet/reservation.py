"""Space-time reservation table (the shared "blackboard" of the airspace).

Cooperative path planning (Silver, 2005) lets agents plan one after another,
each treating the cells already claimed by others as obstacles *in time*.

Two kinds of claims are stored (cells are ``(x, y, z)`` airspace cells):

* vertex ``(cell, t)``  - the drone will be airborne in ``cell`` at tick ``t``
* edge ``(a, b, t)``    - the drone will fly from ``a`` (at ``t``) to ``b``
  (at ``t + 1``). An edge claim blocks the opposite move ``b -> a`` at the same
  tick, which is the classic head-on "swap" collision a vertex check misses.
  Moves between layers are edges too, so a drone climbing through a layer
  that another drone is descending through is a (vertical) head-on conflict.

When ``enabled`` is False every query succeeds and nothing is stored; this is
how the uncoordinated baselines run.
"""

from __future__ import annotations

from collections import defaultdict
from typing import Iterable

from .world import Cell


class ReservationTable:
    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.vertex: dict[tuple[Cell, int], int] = {}
        self.edge: dict[tuple[Cell, Cell, int], int] = {}
        self._owned: dict[int, set] = defaultdict(set)

    # ------------------------------------------------------------------ query
    def vertex_owner(self, c: Cell, t: int) -> int | None:
        return self.vertex.get((c, t))

    def vertex_free(self, c: Cell, t: int, agent: int, ignore: frozenset = frozenset()) -> bool:
        if not self.enabled:
            return True
        o = self.vertex.get((c, t))
        return o is None or o == agent or o in ignore

    def move_free(self, a: Cell, b: Cell, t: int, agent: int, ignore: frozenset = frozenset()) -> bool:
        """Can ``agent`` fly a -> b departing at ``t``? Checks target vertex and swap."""
        if not self.enabled:
            return True
        o = self.vertex.get((b, t + 1))
        if o is not None and o != agent and o not in ignore:
            return False
        if a != b:
            o = self.edge.get((b, a, t))
            if o is not None and o != agent and o not in ignore:
                return False
        return True

    def span_free(self, c: Cell, t0: int, t1: int, agent: int, ignore: frozenset = frozenset()) -> bool:
        """Vertex free for every tick in [t0, t1] (used for hover dwell)."""
        return all(self.vertex_free(c, t, agent, ignore) for t in range(t0, t1 + 1))

    # ---------------------------------------------------------------- mutation
    def reserve_path(self, agent: int, steps: Iterable[tuple[Cell, int, bool]]) -> set[int]:
        """Claim the airborne states of a plan.

        ``steps`` yields ``(cell, t, airborne)``. Returns the ids of any agents
        whose claims were overwritten (only possible during priority escalation);
        those agents must re-plan.
        """
        bumped: set[int] = set()
        if not self.enabled:
            return bumped
        prev = None
        owned = self._owned[agent]
        for c, t, air in steps:
            if air:
                key = (c, t)
                o = self.vertex.get(key)
                if o is not None and o != agent:
                    bumped.add(o)
                self.vertex[key] = agent
                owned.add(("v", key))
                if prev is not None and prev[2] and prev[1] == t - 1 and prev[0] != c:
                    ekey = (prev[0], c, t - 1)
                    self.edge[ekey] = agent
                    owned.add(("e", ekey))
                    # overwrite an opposite claim, if any
                    okey = (c, prev[0], t - 1)
                    o = self.edge.get(okey)
                    if o is not None and o != agent:
                        bumped.add(o)
            prev = (c, t, air)
        return bumped

    def hold(self, agent: int, c: Cell, t0: int, t1: int) -> set[int]:
        """A stuck drone claims its current cell for [t0, t1]; returns bumped agents."""
        return self.reserve_path(agent, ((c, t, True) for t in range(t0, t1 + 1)))

    def release(self, agent: int, from_t: int | None = None) -> None:
        """Drop an agent's claims (all of them, or those at/after ``from_t``)."""
        if not self.enabled:
            return
        owned = self._owned.get(agent)
        if not owned:
            return
        keep = set()
        for kind, key in owned:
            t = key[-1]
            if from_t is not None and t < from_t:
                keep.add((kind, key))
                continue
            table = self.vertex if kind == "v" else self.edge
            if table.get(key) == agent:
                del table[key]
        self._owned[agent] = keep

    def prune(self, before_t: int) -> None:
        """Forget claims in the past to keep the tables small."""
        if not self.enabled:
            return
        for table in (self.vertex, self.edge):
            for key in [k for k in table if k[-1] < before_t]:
                del table[key]
        for agent, owned in self._owned.items():
            self._owned[agent] = {(k, key) for k, key in owned if key[-1] >= before_t}

    def owners_in(self, cells_times: Iterable[tuple[Cell, int]]) -> set[int]:
        return {o for key in cells_times if (o := self.vertex.get(key)) is not None}

    def __len__(self) -> int:
        return len(self.vertex)
