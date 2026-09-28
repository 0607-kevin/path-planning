"""Jump Point Search (JPS).

JPS is an optimization of A* for uniform-cost 8-connected grids. It skips
symmetric, equivalent paths by "jumping" along straight and diagonal lines
until it reaches a *jump point*: the goal, a cell with a forced neighbour, or
(in the diagonal case) a cell whose horizontal/vertical recursion finds one.

The returned path is expanded back into every traversed cell so its length is
directly comparable with plain A*.
"""

from __future__ import annotations

import heapq
from math import inf
from typing import Dict, List, Optional

from .base import BasePlanner, Pos, octile
from utils.grid import EIGHT_DIRS


class JPSPlanner(BasePlanner):
    def __init__(self, grid_map):
        # JPS is defined on 8-connected uniform grids.
        super().__init__(grid_map, diagonal=True)
        self.goal: Pos = (0, 0)

    # ------------------------------------------------------------------
    def plan(self, start: Pos, goal: Pos) -> Optional[List[Pos]]:
        self.reset()
        if not self._valid_endpoints(start, goal):
            return None
        if start == goal:
            return [start]
        self.goal = goal

        g_score: Dict[Pos, float] = {start: 0.0}
        came_from: Dict[Pos, Optional[Pos]] = {start: None}
        dir_from: Dict[Pos, Pos] = {}
        open_set: List[tuple] = [(octile(start, goal), 0, start)]
        closed = set()
        counter = 1

        while open_set:
            _, _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)
            self.nodes_expanded += 1
            self.visited_order.append(current)

            if current == goal:
                return self._expand(self.reconstruct(came_from, current))

            for direction in self._allowed_dirs(current, dir_from.get(current)):
                jp = self._jump(current, direction)
                if jp is None or jp in closed:
                    continue
                ng = g_score[current] + octile(current, jp)
                if ng < g_score.get(jp, inf):
                    g_score[jp] = ng
                    came_from[jp] = current
                    dir_from[jp] = direction
                    heapq.heappush(open_set, (ng + octile(jp, goal), counter, jp))
                    counter += 1

        return None

    # ------------------------------------------------------------------
    def _jump(self, node: Pos, direction: Pos) -> Optional[Pos]:
        dx, dy = direction
        nxt = (node[0] + dx, node[1] + dy)
        if not self.map.is_free(nxt):
            return None
        if dx != 0 and dy != 0:
            # Same no-corner-cutting rule as GridMap.neighbors.
            if not self.map.is_free((node[0] + dx, node[1])):
                return None
            if not self.map.is_free((node[0], node[1] + dy)):
                return None
        if nxt == self.goal:
            return nxt
        if self._has_forced(nxt, direction):
            return nxt
        if dx != 0 and dy != 0:
            # Diagonal move: stop if either cardinal ray finds a jump point.
            if self._jump(nxt, (dx, 0)) is not None:
                return nxt
            if self._jump(nxt, (0, dy)) is not None:
                return nxt
        return self._jump(nxt, direction)

    # ------------------------------------------------------------------
    def _blocked(self, pos: Pos) -> bool:
        return not self.map.is_free(pos)

    def _has_forced(self, node: Pos, direction: Pos) -> bool:
        """A forced neighbour exists when an obstacle next to the node can
        only be reached by passing through this node."""
        x, y = node
        dx, dy = direction

        if dx != 0 and dy != 0:  # diagonal travel
            if self._blocked((x - dx, y)) and self.map.is_free((x - dx, y + dy)):
                return True
            if self._blocked((x, y - dy)) and self.map.is_free((x + dx, y - dy)):
                return True
        elif dx != 0:  # horizontal travel
            if self._blocked((x - dx, y + 1)) and self.map.is_free((x, y + 1)):
                return True
            if self._blocked((x - dx, y - 1)) and self.map.is_free((x, y - 1)):
                return True
        else:  # vertical travel
            if self._blocked((x + 1, y - dy)) and self.map.is_free((x + 1, y)):
                return True
            if self._blocked((x - 1, y - dy)) and self.map.is_free((x - 1, y)):
                return True
        return False

    # ------------------------------------------------------------------
    def _allowed_dirs(self, node: Pos, pdir: Optional[Pos]) -> List[Pos]:
        """Prune neighbours, keeping natural neighbours and forced ones."""
        if pdir is None:
            return list(EIGHT_DIRS)

        x, y = node
        dx, dy = pdir

        if dx != 0 and dy != 0:  # diagonal
            dirs = [pdir, (dx, 0), (0, dy)]
            if self._blocked((x - dx, y)) and self.map.is_free((x - dx, y + dy)):
                dirs.append((-dx, dy))
            if self._blocked((x, y - dy)) and self.map.is_free((x + dx, y - dy)):
                dirs.append((dx, -dy))
        elif dx != 0:  # horizontal: forward + two forward diagonals
            dirs = [(dx, 0), (dx, 1), (dx, -1)]
            if self._blocked((x - dx, y + 1)) and self.map.is_free((x, y + 1)):
                dirs.append((0, 1))
            if self._blocked((x - dx, y - 1)) and self.map.is_free((x, y - 1)):
                dirs.append((0, -1))
        else:  # vertical: forward + two forward diagonals
            dirs = [(0, dy), (1, dy), (-1, dy)]
            if self._blocked((x + 1, y - dy)) and self.map.is_free((x + 1, y)):
                dirs.append((1, 0))
            if self._blocked((x - 1, y - dy)) and self.map.is_free((x - 1, y)):
                dirs.append((-1, 0))
        return dirs

    # ------------------------------------------------------------------
    def _expand(self, path: List[Pos]) -> List[Pos]:
        """Expand a sequence of collinear jump points into a cell-by-cell path."""
        full: List[Pos] = []
        for a, b in zip(path, path[1:]):
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            steps = max(abs(dx), abs(dy))
            sx = dx // steps
            sy = dy // steps
            for k in range(steps):
                full.append((a[0] + sx * k, a[1] + sy * k))
        full.append(path[-1])
        return full
