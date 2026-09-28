"""Matplotlib-based visualisation of searches and resulting paths."""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .grid import GridMap

Pos = tuple


def _draw_map(ax, grid_map: GridMap):
    ax.imshow(
        grid_map.grid,
        cmap="gray_r",
        origin="lower",
        extent=(0, grid_map.width, 0, grid_map.height),
        interpolation="nearest",
        vmin=0,
        vmax=1,
    )
    ax.set_xticks(range(grid_map.width + 1), minor=True)
    ax.set_yticks(range(grid_map.height + 1), minor=True)
    ax.grid(which="minor", color="0.85", linewidth=0.4)
    ax.set_xlim(0, grid_map.width)
    ax.set_ylim(0, grid_map.height)
    ax.set_aspect("equal")


def _draw_endpoints(ax, start: Pos, goal: Pos):
    ax.scatter(start[0] + 0.5, start[1] + 0.5, marker="o",
               color="#2e7d32", s=90, zorder=5, label="start")
    ax.scatter(goal[0] + 0.5, goal[1] + 0.5, marker="*",
               color="#c62828", s=170, zorder=5, label="goal")


def plot_path(grid_map: GridMap, path: Optional[Sequence[Pos]],
              start: Pos, goal: Pos, title: str = "Path",
              visited: Optional[Sequence[Pos]] = None, ax=None):
    """Static plot of the explored region (optional) and final path."""
    created = ax is None
    if created:
        fig, ax = plt.subplots(figsize=(8, 8))
    _draw_map(ax, grid_map)

    if visited:
        vx = [p[0] + 0.5 for p in visited]
        vy = [p[1] + 0.5 for p in visited]
        ax.scatter(vx, vy, s=18, color="#90caf9", alpha=0.5, label="explored")

    if path:
        px = [p[0] + 0.5 for p in path]
        py = [p[1] + 0.5 for p in path]
        ax.plot(px, py, color="#1565c0", linewidth=2.2, label="path")

    _draw_endpoints(ax, start, goal)
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    return ax


def animate_search(grid_map: GridMap, planner, start: Pos, goal: Pos,
                   interval: int = 20, title: str = "Search"):
    """Animate the cell expansion order, then draw the final path."""
    path = planner.plan(start, goal)
    visited = planner.visited_order

    fig, ax = plt.subplots(figsize=(8, 8))
    _draw_map(ax, _map := grid_map)
    _draw_endpoints(ax, start, goal)

    explored_scatter = ax.scatter([], [], s=20, color="#90caf9",
                                  alpha=0.6, label="explored")
    (path_line,) = ax.plot([], [], color="#1565c0", linewidth=2.2, label="path")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)

    n = len(visited)

    def update(frame):
        if frame < n:
            pts = visited[: frame + 1]
            explored_scatter.set_offsets([(p[0] + 0.5, p[1] + 0.5) for p in pts])
            path_line.set_data([], [])
        elif path:
            explored_scatter.set_offsets([(p[0] + 0.5, p[1] + 0.5) for p in visited])
            path_line.set_data(
                [p[0] + 0.5 for p in path], [p[1] + 0.5 for p in path]
            )
        return explored_scatter, path_line

    anim = FuncAnimation(fig, update, frames=n + 1, interval=interval,
                         blit=True, repeat=False)
    return fig, anim


def plot_comparison(grid_map: GridMap, results: Dict[str, dict],
                    start: Pos, goal: Pos):
    """Plot every planner's path side by side.

    ``results`` maps a display name to ``{"path": ..., "visited": ...}``.
    """
    n = len(results)
    cols = min(3, n)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 5.2 * rows),
                             squeeze=False)
    for ax, (name, res) in zip(axes.flat, results.items()):
        path = res["path"]
        length = (
            sum(grid_map.step_cost(a, b) for a, b in zip(path, path[1:]))
            if path else None
        )
        subtitle = f"{name} | length={length:.2f}, expanded={res.get('expanded', '?')}"
        plot_path(grid_map, path, start, goal, subtitle,
                  visited=res.get("visited"), ax=ax)
    for ax in axes.flat[n:]:
        ax.axis("off")
    fig.tight_layout()
    return fig
