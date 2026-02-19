#!/usr/bin/env python3
"""Shared helpers for goal placement and geometry. Used by Task1, Task2, Task3."""

from __future__ import annotations
from typing import List, Tuple

import numpy as np

Vec2 = Tuple[float, float]

# Default clearance from obstacles (m) so goals are not placed on/near black buoys
DEFAULT_GOAL_CLEARANCE = 5.0


def segments_intersect(p1: Vec2, p2: Vec2, q1: Vec2, q2: Vec2) -> bool:
    """True if segment p1->p2 intersects segment q1->q2. Used for gate crossing (Task1, Task3)."""
    a = np.array(p1, dtype=float)
    b = np.array(p2, dtype=float)
    c = np.array(q1, dtype=float)
    d = np.array(q2, dtype=float)

    def _orient(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> float:
        return float((y[0] - x[0]) * (z[1] - x[1]) - (y[1] - x[1]) * (z[0] - x[0]))

    def _on_segment(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> bool:
        return (
            min(x[0], y[0]) - 1e-9 <= z[0] <= max(x[0], y[0]) + 1e-9
            and min(x[1], y[1]) - 1e-9 <= z[1] <= max(x[1], y[1]) + 1e-9
        )

    o1, o2 = _orient(a, b, c), _orient(a, b, d)
    o3, o4 = _orient(c, d, a), _orient(c, d, b)
    if (o1 * o2 < 0.0) and (o3 * o4 < 0.0):
        return True
    if abs(o1) < 1e-9 and _on_segment(a, b, c): return True
    if abs(o2) < 1e-9 and _on_segment(a, b, d): return True
    if abs(o3) < 1e-9 and _on_segment(c, d, a): return True
    if abs(o4) < 1e-9 and _on_segment(c, d, b): return True
    return False


def nudge_goal_away_from_obstacles(
    desired: Vec2,
    obstacles: List[Vec2],
    from_point: Vec2,
    min_clearance: float = DEFAULT_GOAL_CLEARANCE,
    step: float = 1.0,
) -> Vec2:
    """Return a goal at least min_clearance from all obstacles, keeping the same direction from from_point.

    If the desired position is too close to any obstacle, we slide the goal back toward from_point
    until it is clear. This preserves "correct direction" (from_point -> goal) while avoiding
    placing a waypoint on or near a black buoy.

    Args:
        desired: Preferred goal position (e.g. gate center, 50 m ahead).
        obstacles: Obstacle positions in meters (e.g. from entities.get_obstacles() + get_no_go_obstacle_points()).
        from_point: Reference point in meters (e.g. current boat position); direction from_point -> desired is preserved.
        min_clearance: Minimum distance (m) from any obstacle.
        step: Step size (m) when sliding back.

    Returns:
        A position at least min_clearance from all obstacles, or desired if no obstacles / already clear.
    """
    if not obstacles:
        return desired

    desired_arr = np.array(desired, dtype=float)
    from_arr = np.array(from_point, dtype=float)
    obs_arrs = [np.array(o, dtype=float) for o in obstacles]
    dir_back = from_arr - desired_arr
    dist_back = float(np.linalg.norm(dir_back))
    if dist_back < 1e-6:
        return desired
    dir_back = dir_back / dist_back

    candidate = desired_arr.copy()
    while True:
        if all(float(np.linalg.norm(candidate - o)) >= min_clearance for o in obs_arrs):
            return (float(candidate[0]), float(candidate[1]))
        candidate = candidate + dir_back * step
        if float(np.linalg.norm(candidate - from_arr)) <= step:
            return (float(candidate[0]), float(candidate[1]))
