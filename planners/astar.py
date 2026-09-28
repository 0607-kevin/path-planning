"""A* search with a selectable, optionally weighted heuristic."""

from __future__ import annotations

import heapq
from math import inf
from typing import Dict, List, Optional

from .base import HEURISTICS, BasePlanner, Pos


class AStarPlanner(BasePlanner):
    """Classic A* search.

    Parameters
    ----------
    grid_map:
        Occupancy grid to search.
    diagonal:
        Allow 8-connected movement.
    heuristic:
        One of ``"manhattan"``, ``"euclidean"`` or ``"octile"``. Defaults to
        a consistent heuristic matching the connectivity mode.
    weight:
        Heuristic weight (``1.0`` = standard optimal A*, ``>1`` = weighted A*,
        faster but potentially suboptimal).
    """

    def __init__(
        self,
        grid_map,
        diagonal: bool = False,
        heuristic: Optional[str] = None,
        weight: float = 1.0,
    ):
        super().__init__(grid_map, diagonal)
        if heuristic is None:
            heuristic = "octile" if diagonal else "manhattan"
        if heuristic not in HEURISTICS:
            raise ValueError(f"unknown heuristic: {heuristic}")
        if weight < 1.0:
            raise ValueError("weight must be >= 1.0 to guarantee optimality")
        self.heuristic = HEURISTICS[heuristic]
        self.weight = weight

    def plan(self, start: Pos, goal: Pos) -> Optional[List[Pos]]:
        self.reset()
        if not self._valid_endpoints(start, goal):
            return None
        if start == goal:
            return [start]

        g_score: Dict[Pos, float] = {start: 0.0}
        came_from: Dict[Pos, Optional[Pos]] = {start: None}
        open_set: List[tuple] = [
            (self.weight * self.heuristic(start, goal), 0, start)
        ]
        closed = set()
        counter = 1  # tie-breaker so tuples never compare cells

        while open_set:
            _, _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)
            self.nodes_expanded += 1
            self.visited_order.append(current)

            if current == goal:
                return self.reconstruct(came_from, current)

            g = g_score[current]
            for nxt in self.map.neighbors(current, self.diagonal):
                if nxt in closed:
                    continue
                tentative = g + self.step_cost(current, nxt)
                if tentative < g_score.get(nxt, inf):
                    g_score[nxt] = tentative
                    came_from[nxt] = current
                    f = tentative + self.weight * self.heuristic(nxt, goal)
                    heapq.heappush(open_set, (f, counter, nxt))
                    counter += 1

        return None
