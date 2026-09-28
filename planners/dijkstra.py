"""Dijkstra's shortest-path algorithm (uniform-cost search)."""

from __future__ import annotations

import heapq
from math import inf
from typing import Dict, List, Optional

from .base import BasePlanner, Pos


class DijkstraPlanner(BasePlanner):
    def plan(self, start: Pos, goal: Pos) -> Optional[List[Pos]]:
        self.reset()
        if not self._valid_endpoints(start, goal):
            return None
        if start == goal:
            return [start]

        dist: Dict[Pos, float] = {start: 0.0}
        came_from: Dict[Pos, Optional[Pos]] = {start: None}
        open_set: List[tuple] = [(0.0, start)]
        closed = set()

        while open_set:
            g, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)
            self.nodes_expanded += 1
            self.visited_order.append(current)

            if current == goal:
                return self.reconstruct(came_from, current)

            for nxt in self.map.neighbors(current, self.diagonal):
                if nxt in closed:
                    continue
                tentative = g + self.step_cost(current, nxt)
                if tentative < dist.get(nxt, inf):
                    dist[nxt] = tentative
                    came_from[nxt] = current
                    heapq.heappush(open_set, (tentative, nxt))

        return None
