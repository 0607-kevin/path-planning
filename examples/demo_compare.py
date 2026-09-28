"""Compare BFS, Dijkstra, Greedy, A* and JPS on the same random map."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt

from planners import (
    AStarPlanner,
    BFSPlanner,
    DijkstraPlanner,
    GreedyPlanner,
    JPSPlanner,
)
from utils import GridMap, plot_comparison


def main() -> None:
    grid = GridMap.random_map(40, 28, obstacle_ratio=0.22, seed=7)
    start, goal = (2, 2), (37, 25)
    # Make sure the endpoints are free.
    grid.carve_circle(start, 1)
    grid.carve_circle(goal, 1)

    planners = {
        "BFS (4-dir)": BFSPlanner(grid, diagonal=False),
        "Dijkstra": DijkstraPlanner(grid, diagonal=True),
        "Greedy": GreedyPlanner(grid, diagonal=True),
        "A*": AStarPlanner(grid, diagonal=True),
        "JPS": JPSPlanner(grid),
    }

    results = {}
    print(f"{'planner':<12}{'length':>10}{'expanded':>10}")
    for name, planner in planners.items():
        path = planner.plan(start, goal)
        length = (
            sum(grid.step_cost(a, b) for a, b in zip(path, path[1:]))
            if path else float("nan")
        )
        results[name] = {
            "path": path,
            "visited": planner.visited_order,
            "expanded": planner.nodes_expanded,
        }
        print(f"{name:<12}{length:>10.2f}{planner.nodes_expanded:>10}")

    plot_comparison(grid, results, start, goal)
    plt.show()


if __name__ == "__main__":
    main()
