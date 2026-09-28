"""Common interfaces shared by every grid-based planner."""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from utils.grid import SQRT2, GridMap

Pos = Tuple[int, int]


class BasePlanner:
    """Abstract base class for grid search planners.

    Attributes
    ----------
    map:
        The :class:`~utils.grid.GridMap` used for the search.
    diagonal:
        Whether 8-connected (diagonal) moves are allowed.
    visited_order:
        Cells in the order they were expanded by the search. Useful for
        visualising the explored region.
    nodes_expanded:
        Number of cells popped from the search frontier.
    """

    def __init__(self, grid_map: GridMap, diagonal: bool = False):
        self.map = grid_map
        self.diagonal = diagonal
        self.visited_order: List[Pos] = []
        self.nodes_expanded = 0

    # ------------------------------------------------------------------
    def step_cost(self, a: Pos, b: Pos) -> float:
        return self.map.step_cost(a, b)

    def plan(self, start: Pos, goal: Pos) -> Optional[List[Pos]]:
        """Search a path from ``start`` to ``goal``.

        Returns a list of cells from start to goal, or ``None`` when no path
        exists.
        """
        raise NotImplementedError

    def reset(self) -> None:
        self.visited_order = []
        self.nodes_expanded = 0

    # ------------------------------------------------------------------
    @staticmethod
    def reconstruct(came_from: Dict[Pos, Optional[Pos]], current: Pos) -> List[Pos]:
        """Walk the ``came_from`` parent chain back to the start."""
        path = [current]
        while came_from.get(current) is not None:
            current = came_from[current]  # type: ignore[assignment]
            path.append(current)
        path.reverse()
        return path

    # ------------------------------------------------------------------
    def _valid_endpoints(self, start: Pos, goal: Pos) -> bool:
        return self.map.is_free(start) and self.map.is_free(goal)


def manhattan(a: Pos, b: Pos) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Pos, b: Pos) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def octile(a: Pos, b: Pos) -> float:
    """Admissible heuristic for 8-connected grids."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy) + (SQRT2 - 1) * min(dx, dy)


HEURISTICS = {
    "manhattan": manhattan,
    "euclidean": euclidean,
    "octile": octile,
}
