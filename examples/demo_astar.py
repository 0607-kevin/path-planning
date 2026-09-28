"""A* on a hand-built map with several wall segments."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt

from planners import AStarPlanner
from utils import GridMap, plot_path


def build_map() -> GridMap:
    m = GridMap(25, 18)
    m.add_border()
    m.add_rect(6, 2, 2, 10)
    m.add_rect(12, 6, 2, 10)
    m.add_rect(18, 2, 2, 9)
    return m


def main() -> None:
    grid = build_map()
    start, goal = (2, 2), (22, 15)

    planner = AStarPlanner(grid, diagonal=True)
    path = planner.plan(start, goal)
    assert path is not None, "no path found"

    length = sum(grid.step_cost(a, b) for a, b in zip(path, path[1:]))
    print(f"A* path length : {length:.2f}")
    print(f"nodes expanded : {planner.nodes_expanded}")

    plot_path(grid, path, start, goal, "A* (8-connected)",
              visited=planner.visited_order)
    plt.show()


if __name__ == "__main__":
    main()
