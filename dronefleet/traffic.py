"""Reactive safety layer and collision detection.

Cooperative planning makes plans conflict-free *on paper*. Execution is not
perfect: wind gusts hold drones back, and the uncoordinated baselines do not
plan around each other at all. Every tick, before anything moves, each drone
broadcasts its intended next cell (like ADS-B / V2V) and applies a local
right-of-way rule:

1. A drone that is staying put (hovering) always keeps its cell.
2. If several drones want the same cell, airborne traffic beats a drone
   that is about to take off (waiting on the ground is free and safe), then
   the highest-ranked one gets it (rank = carrying an express parcel >
   carrying any parcel > empty; ties by id); the others hold position.
3. Two drones that would swap cells head-on: the lower-ranked one holds -
   which blocks the other too, so a head-on pair always stops (re-planning,
   not the reflex layer, has to resolve it).
4. A drone forced to hold occupies its current cell, which may in turn block
   someone else - so the rules are re-applied until nothing changes.

Each pass only ever turns movers into holders, so the loop terminates, and the
fixed point is collision-free as long as the current positions are (drones
on distinct cells stay on distinct cells). The simulation computes this fixed
point centrally for speed; it is the same outcome each drone reaches by
applying the rule to its neighbours' broadcasts.
"""

from __future__ import annotations

from collections import defaultdict

from .world import Cell

# intent: did -> (cur_cell, cur_air, tgt_cell, tgt_air)
Intent = tuple[Cell, bool, Cell, bool]


def resolve(intents: dict[int, Intent], rank: dict[int, tuple]) -> tuple[dict[int, Intent], set[int]]:
    """Return (final intents, ids that were forced to hold)."""
    final = dict(intents)
    forced: set[int] = set()

    def moving(i: int) -> bool:
        cc, ca, tc, ta = final[i]
        return ta and (tc != cc or not ca)

    def hold(i: int) -> None:
        cc, ca, tc, ta = final[i]
        final[i] = (cc, ca, cc, ca)     # stay (in the air, or on the ground if taking off)
        forced.add(i)

    occupant = {v[0]: i for i, v in intents.items() if v[1]}
    changed = True
    while changed:
        changed = False
        claims: dict[Cell, list[int]] = defaultdict(list)
        for i, (cc, ca, tc, ta) in final.items():
            if ta:
                claims[tc].append(i)
        for cell, ids in claims.items():
            if len(ids) < 2:
                continue
            stayers = [i for i in ids if not moving(i)]
            winner = stayers[0] if stayers else max(ids, key=lambda i: (final[i][1], rank[i]))
            for i in ids:
                if i != winner:
                    hold(i)
                    changed = True
        for i in list(final):
            if not moving(i):
                continue
            cc, ca, tc, ta = final[i]
            if not ca:
                continue
            j = occupant.get(tc)
            if j is None or j == i:
                continue
            jc, ja, jt, jta = final[j]
            if jta and jt == cc and ja:
                loser = i if rank[i] < rank[j] else j
                hold(loser)
                changed = True
    return final, forced


def detect_collisions(before: dict[int, tuple[Cell, bool]], after: dict[int, tuple[Cell, bool]]) -> list[tuple]:
    """Vertex (same cell) and edge (head-on swap) conflicts between airborne drones."""
    out = []
    cells: dict[Cell, list[int]] = defaultdict(list)
    for i, (c, air) in after.items():
        if air:
            cells[c].append(i)
    for c, ids in cells.items():
        if len(ids) > 1:
            out.append(("vertex", tuple(sorted(ids)), c))
    moved = {i: (before[i][0], after[i][0]) for i in after
             if after[i][1] and before[i][1] and before[i][0] != after[i][0]}
    by_edge = {v: i for i, v in moved.items()}
    for i, (a, b) in moved.items():
        j = by_edge.get((b, a))
        if j is not None and i < j:
            out.append(("edge", (i, j), a))
    return out
