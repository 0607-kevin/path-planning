"""Animated A* search: watch the explored region grow and the path appear."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt

from planners import AStarPlanner
from utils import GridMap, animate_search


def main() -> None:
    grid = GridMap.random_map(35, 25, obstacle_ratio=0.2, seed=3)
    start, goal = (2, 2), (32, 22)
    grid.carve_circle(start, 1)
    grid.carve_circle(goal, 1)

    planner = AStarPlanner(grid, diagonal=True)
    _, anim = animate_search(grid, planner, start, goal,
                             interval=15, title="A* search animation")
    # Keep a reference so the animation is not garbage-collected.
    plt.show()


if __name__ == "__main__":
    main()
