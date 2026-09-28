"""2D occupancy grid map utilities.

The map is a 2D integer matrix where ``0`` denotes a free cell and ``1``
denotes an occupied cell. Coordinates are expressed as ``(x, y)`` tuples with
``x`` being the column index and ``y`` the row index.
"""

from __future__ import annotations

import random
from typing import Iterator, List, Optional, Tuple

Pos = Tuple[int, int]

# 4-connected and 8-connected movement directions
FOUR_DIRS: List[Pos] = [(1, 0), (-1, 0), (0, 1), (0, -1)]
EIGHT_DIRS: List[Pos] = FOUR_DIRS + [
    (1, 1), (1, -1), (-1, 1), (-1, -1),
]

SQRT2 = 1.4142135623730951


class GridMap:
    """A simple 2D occupancy grid."""

    def __init__(self, width: int, height: int):
        if width <= 0 or height <= 0:
            raise ValueError("width and height must be positive")
        self.width = width
        self.height = height
        self.grid: List[List[int]] = [[0] * width for _ in range(height)]

    # ------------------------------------------------------------------
    # Basic queries
    # ------------------------------------------------------------------
    def in_bounds(self, pos: Pos) -> bool:
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, pos: Pos) -> bool:
        x, y = pos
        return self.in_bounds(pos) and self.grid[y][x] == 0

    def is_obstacle(self, pos: Pos) -> bool:
        return self.in_bounds(pos) and self.grid[pos[1]][pos[0]] == 1

    # ------------------------------------------------------------------
    # Map construction
    # ------------------------------------------------------------------
    def set_obstacle(self, pos: Pos) -> None:
        x, y = pos
        if self.in_bounds(pos):
            self.grid[y][x] = 1

    def set_free(self, pos: Pos) -> None:
        x, y = pos
        if self.in_bounds(pos):
            self.grid[y][x] = 0

    def add_rect(self, x: int, y: int, w: int, h: int) -> None:
        """Add a filled rectangular obstacle."""
        for ry in range(y, y + h):
            for rx in range(x, x + w):
                self.set_obstacle((rx, ry))

    def add_border(self) -> None:
        """Surround the map with a one-cell thick obstacle border."""
        for x in range(self.width):
            self.set_obstacle((x, 0))
            self.set_obstacle((x, self.height - 1))
        for y in range(self.height):
            self.set_obstacle((0, y))
            self.set_obstacle((self.width - 1, y))

    # ------------------------------------------------------------------
    # Neighbour expansion
    # ------------------------------------------------------------------
    def neighbors(self, pos: Pos, diagonal: bool = False) -> List[Pos]:
        """Return traversable neighbours of ``pos``.

        Diagonal moves are only allowed when both adjacent cardinal cells are
        free, which prevents the path from cutting through obstacle corners.
        """
        x, y = pos
        dirs = EIGHT_DIRS if diagonal else FOUR_DIRS
        result: List[Pos] = []
        for dx, dy in dirs:
            nxt = (x + dx, y + dy)
            if not self.is_free(nxt):
                continue
            if dx != 0 and dy != 0:
                # Disallow corner cutting between two blocked cardinal cells.
                if not self.is_free((x + dx, y)) or not self.is_free((x, y + dy)):
                    continue
            result.append(nxt)
        return result

    def step_cost(self, a: Pos, b: Pos) -> float:
        """Euclidean step cost between two adjacent cells."""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if dx == 1 and dy == 1:
            return SQRT2
        return 1.0

    def __iter__(self) -> Iterator[Pos]:
        for y in range(self.height):
            for x in range(self.width):
                yield (x, y)

    # ------------------------------------------------------------------
    # Factories
    # ------------------------------------------------------------------
    @classmethod
    def random_map(
        cls,
        width: int,
        height: int,
        obstacle_ratio: float = 0.2,
        seed: Optional[int] = None,
        border: bool = True,
    ) -> "GridMap":
        """Create a map with randomly scattered obstacles.

        The border (if requested) and a small clear area around the map centre
        are kept free so that planning queries are more likely to be solvable.
        """
        rng = random.Random(seed)
        m = cls(width, height)
        for pos in m:
            if rng.random() < obstacle_ratio:
                m.set_obstacle(pos)
        if border:
            m.add_border()
        return m

    def carve_circle(self, center: Pos, radius: int) -> None:
        """Clear every cell within ``radius`` (Chebyshev distance) of centre."""
        cx, cy = center
        for y in range(cy - radius, cy + radius + 1):
            for x in range(cx - radius, cx + radius + 1):
                self.set_free((x, y))
