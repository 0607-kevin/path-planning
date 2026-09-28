"""Breadth-First Search (unweighted, optimal with unit edge costs)."""

from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional

from .base import BasePlanner, Pos


class BFSPlanner(BasePlanner):
    def plan(self, start: Pos, goal: Pos) -> Optional[List[Pos]]:
        self.reset()
        if not self._valid_endpoints(start, goal):
            return None
        if start == goal:
            return [start]

        queue: deque[Pos] = deque([start])
        came_from: Dict[Pos, Optional[Pos]] = {start: None}

        while queue:
            current = queue.popleft()
            self.nodes_expanded += 1
            self.visited_order.append(current)

            if current == goal:
                return self.reconstruct(came_from, current)

            for nxt in self.map.neighbors(current, self.diagonal):
                if nxt not in came_from:
                    came_from[nxt] = current
                    queue.append(nxt)

        return None
