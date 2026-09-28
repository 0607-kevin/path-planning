# path-planning

![Python](https://img.shields.io/badge/Python-3.9%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Tests](https://img.shields.io/badge/tests-7%20passed-brightgreen)

A collection of classic **2D grid path planning algorithms** implemented from
scratch in Python, with matplotlib visualisation, search animations and a
side-by-side comparison of path quality and search effort.

## Algorithms

| Planner | Type | Optimal | Remarks |
|---------|------|---------|---------|
| **BFS** | uninformed | yes (unit cost) | Expands uniformly; shortest in number of steps |
| **Dijkstra** | uninformed | yes | Uniform-cost search; works with arbitrary edge weights |
| **Greedy Best-First** | heuristic | no | Expands the cell closest to the goal; very fast but can be trapped |
| **A\*** | heuristic | yes | Best-first search with `f = g + h`; supports weighted A* |
| **JPS** | heuristic | yes | Jump Point Search; A* optimisation that prunes symmetric paths |

Three admissible heuristics are provided: Manhattan, Euclidean and Octile
(the last being consistent for 8-connected grids). Diagonal movement forbids
cutting through obstacle corners.

## Project layout

```
path-planning/
├── planners/
│   ├── base.py         # shared interface + heuristics
│   ├── bfs.py
│   ├── dijkstra.py
│   ├── greedy.py
│   ├── astar.py
│   └── jps.py          # Jump Point Search
├── utils/
│   ├── grid.py         # occupancy grid, random maps
│   └── visualize.py    # static plots + search animation
├── examples/
│   ├── demo_astar.py
│   ├── demo_compare.py
│   ├── demo_jps.py
│   └── demo_animated.py
└── tests/
    └── test_planners.py
```

## Installation

```bash
git clone https://github.com/0607-kevin/path-planning.git
cd path-planning
pip install -r requirements.txt
```

## Quick start

```python
from planners import AStarPlanner
from utils import GridMap

grid = GridMap.random_map(40, 28, obstacle_ratio=0.22, seed=7)
start, goal = (2, 2), (37, 25)

planner = AStarPlanner(grid, diagonal=True)
path = planner.plan(start, goal)
print(len(path), planner.nodes_expanded)
```

Run the examples:

```bash
python examples/demo_astar.py       # single A* query
python examples/demo_compare.py     # compare all planners
python examples/demo_jps.py         # A* vs JPS node expansion
python examples/demo_animated.py    # animated search
```

## Sample comparison

On a 40×28 random map (22% obstacles), `examples/demo_compare.py` produces:

```
planner         length  expanded
BFS (4-dir)      58.00       747
Dijkstra         47.46       743
Greedy           52.87        55
A*               47.46       267
JPS              47.46       117
```

Dijkstra, A* and JPS all return the same optimal path length, while JPS
expands the fewest nodes. Greedy is fastest but suboptimal; BFS without
diagonal moves produces the longest (step-count optimal) path.

## Tests

```bash
python tests/test_planners.py
# or
python -m pytest tests/ -v
```

The suite checks path continuity, failure cases, A*/Dijkstra optimality,
weighted A* speed-up, and verifies on 20 random maps that JPS returns the same
optimal length as A* while never expanding more nodes.

## References

- Dijkstra, E. W. (1959). *A note on two problems in connexion with graphs.*
- Hart, P. E., Nilsson, N. J., Raphael, B. (1968). *A Formal Basis for the
  Heuristic Determination of Minimum Cost Paths.*
- Harabor, D., Grastien, A. (2011). *Online Graph Pruning for Pathfinding on
  Grid Maps.* (Jump Point Search)

## License

Released under the [MIT License](LICENSE).
