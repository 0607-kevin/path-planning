"""Compare the number of nodes expanded by A* and JPS."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt

from planners import AStarPlanner, JPSPlanner
from utils import GridMap, plot_comparison


def main() -> None:
    grid = GridMap.random_map(45, 30, obstacle_ratio=0.25, seed=42)
    start, goal = (2, 2), (42, 27)
    grid.carve_circle(start, 1)
    grid.carve_circle(goal, 1)

    astar = AStarPlanner(grid, diagonal=True)
    jps = JPSPlanner(grid)

    results = {}
    for name, planner in (("A*", astar), ("JPS", jps)):
        path = planner.plan(start, goal)
        results[name] = {
            "path": path,
            "visited": planner.visited_order,
            "expanded": planner.nodes_expanded,
        }
        length = sum(grid.step_cost(a, b) for a, b in zip(path, path[1:]))
        print(f"{name}: length={length:.2f}, expanded={planner.nodes_expanded}")

    speedup = astar.nodes_expanded / max(jps.nodes_expanded, 1)
    print(f"JPS expanded {speedup:.1f}x fewer nodes than A*")

    plot_comparison(grid, results, start, goal)
    plt.show()


if __name__ == "__main__":
    main()
