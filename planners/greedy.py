"""Greedy Best-First Search (expands the cell closest to the goal).

Greedy search only considers the heuristic value. It is fast but neither
optimal nor complete when edge costs vary.
"""

from __future__ import annotations

import heapq
from typing import Dict, List, Optional

from .base import HEURISTICS, BasePlanner, Pos


class GreedyPlanner(BasePlanner):
    def __init__(self, grid_map, diagonal: bool = False, heuristic: str = "octile"):
        super().__init__(grid_map, diagonal)
        if heuristic not in HEURISTICS:
            raise ValueError(f"unknown heuristic: {heuristic}")
        self.heuristic = HEURISTICS[heuristic]

    def plan(self, start: Pos, goal: Pos) -> Optional[List[Pos]]:
        self.reset()
        if not self._valid_endpoints(start, goal):
            return None
        if start == goal:
            return [start]

        open_set: List[tuple] = [(self.heuristic(start, goal), start)]
        came_from: Dict[Pos, Optional[Pos]] = {start: None}
        closed = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)
            self.nodes_expanded += 1
            self.visited_order.append(current)

            if current == goal:
                return self.reconstruct(came_from, current)

            for nxt in self.map.neighbors(current, self.diagonal):
                if nxt not in closed and nxt not in came_from:
                    came_from[nxt] = current
                    heapq.heappush(
                        open_set, (self.heuristic(nxt, goal), nxt)
                    )

        return None
