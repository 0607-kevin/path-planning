"""Tests for the grid planners.

Run from the project root with::

    python -m pytest tests/ -v

or simply::

    python tests/test_planners.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from planners import (
    AStarPlanner,
    BFSPlanner,
    DijkstraPlanner,
    GreedyPlanner,
    JPSPlanner,
)
from utils import GridMap


def walled_map() -> GridMap:
    """A map whose only opening forces every planner to detour."""
    m = GridMap(20, 12)
    m.add_border()
    # Vertical wall with a gap near the bottom.
    m.add_rect(10, 1, 1, 8)
    return m


def path_length(grid: GridMap, path) -> float:
    return sum(grid.step_cost(a, b) for a, b in zip(path, path[1:]))


def assert_continuous(grid: GridMap, path, diagonal: bool) -> None:
    for a, b in zip(path, path[1:]):
        assert b in grid.neighbors(a, diagonal), f"non-adjacent step {a}->{b}"


def test_all_planners_find_a_path():
    grid = walled_map()
    start, goal = (2, 6), (17, 6)
    planners = [
        BFSPlanner(grid, diagonal=False),
        BFSPlanner(grid, diagonal=True),
        DijkstraPlanner(grid, diagonal=True),
        GreedyPlanner(grid, diagonal=True),
        AStarPlanner(grid, diagonal=True),
        JPSPlanner(grid),
    ]
    for planner in planners:
        path = planner.plan(start, goal)
        assert path is not None, f"{planner.__class__.__name__} failed"
        assert path[0] == start and path[-1] == goal
        assert_continuous(grid, path, diagonal=True)


def test_no_path_returns_none():
    grid = GridMap(10, 10)
    grid.add_border()
    # A full vertical wall (y = 1..8, no gap) splits the map in two.
    grid.add_rect(4, 1, 1, 8)
    planner = AStarPlanner(grid, diagonal=True)
    assert planner.plan((1, 1), (8, 8)) is None


def test_blocked_endpoint_returns_none():
    grid = GridMap(8, 8)  # no border, every cell free by default
    planner = AStarPlanner(grid)
    grid.set_obstacle((7, 7))
    assert planner.plan((1, 1), (7, 7)) is None  # goal blocked
    grid.set_obstacle((1, 1))
    assert planner.plan((1, 1), (6, 6)) is None  # start blocked


def test_astar_matches_dijkstra_optimal_length():
    grid = GridMap.random_map(35, 25, obstacle_ratio=0.2, seed=11)
    start, goal = (1, 1), (33, 23)
    grid.carve_circle(start, 1)
    grid.carve_circle(goal, 1)

    dijkstra = DijkstraPlanner(grid, diagonal=True)
    astar = AStarPlanner(grid, diagonal=True)
    p1 = dijkstra.plan(start, goal)
    p2 = astar.plan(start, goal)
    assert p1 is not None and p2 is not None
    assert abs(path_length(grid, p1) - path_length(grid, p2)) < 1e-9


def test_jps_matches_astar_on_random_maps():
    for seed in range(20):
        grid = GridMap.random_map(30, 20, obstacle_ratio=0.22, seed=seed)
        start, goal = (1, 1), (28, 18)
        grid.carve_circle(start, 1)
        grid.carve_circle(goal, 1)

        astar = AStarPlanner(grid, diagonal=True)
        jps = JPSPlanner(grid)
        pa = astar.plan(start, goal)
        pj = jps.plan(start, goal)
        if pa is None:
            assert pj is None, f"seed {seed}: A* found nothing but JPS did"
            continue
        assert pj is not None, f"seed {seed}: JPS missed an existing path"
        assert abs(path_length(grid, pa) - path_length(grid, pj)) < 1e-9, (
            f"seed {seed}: JPS length {path_length(grid, pj)} != "
            f"A* length {path_length(grid, pa)}"
        )
        # JPS should never expand more nodes than plain A*.
        assert jps.nodes_expanded <= astar.nodes_expanded


def test_bfs_finds_minimum_cardinal_steps():
    grid = walled_map()
    start, goal = (2, 6), (17, 6)
    bfs = BFSPlanner(grid, diagonal=False)
    path = bfs.plan(start, goal)
    assert path is not None
    assert len(path) - 1 == abs(goal[0] - start[0]) + abs(goal[1] - start[1]) + 6


def test_weighted_astar_is_faster_or_equal():
    grid = GridMap.random_map(40, 28, obstacle_ratio=0.25, seed=99)
    start, goal = (1, 1), (38, 26)
    grid.carve_circle(start, 1)
    grid.carve_circle(goal, 1)

    plain = AStarPlanner(grid, diagonal=True, weight=1.0)
    weighted = AStarPlanner(grid, diagonal=True, weight=3.0)
    plain.plan(start, goal)
    weighted.plan(start, goal)
    assert weighted.nodes_expanded <= plain.nodes_expanded


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"\n{len(fns)} tests passed.")


if __name__ == "__main__":
    _run_all()
