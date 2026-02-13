


#!/usr/bin/env python3
"""Global Planner Task 3 (Speed Challenge):

What this module does (Task 3):
- Input: EntityList (types + positions) from JSON/perception
- Output: Updated EntityList with ORDERED goal waypoints:
    goal_wp1, goal_wp2, ...

Task objective:
- Rapidly complete a speed challenge by:
    1) Passing through the entrance gate (red + green buoys)
    2) Circling a yellow buoy in the correct direction indicated
       by a separate color indicator buoy
    3) Exiting back through the same gate as quickly as possible

Key rules implemented:
1) Gate identification and ordering:
   - Identify the gate using paired (red_buoy, green_buoy) 
   - Compute gate center and store the waypoint for later return
   - Use boat heading or start-to-gate vector to define forward direction

2) Gate traversal logic:
   - Gate is considered passed when the boat motion segment
     (prev_pos -> current_pos) intersects the gate segment
   - Gate must be crossed once on entry and once on exit

3) Indicator-based maneuver planning:
   - Detect color indicator buoy (red or green)
   - Red indicator  -> start a counter-clockwise (right-side) loop
   - Green indicator -> start a clockwise (left-side) loop
   - Generate left or right waypoint until yellow buoy is discovered

4) yellow buoy is discovered so Speed-optimized waypoint generation:
   - Create waypoint just beyond the yellow buoy in the tangential and final way point being the gate stored from 1)
   - Prefer smooth, continuous trajectories for high-speed execution

5) Task completion tracking:
   - Start timing when the entrance gate is first crossed
   - Stop timing when the exit gate is crossed after circling the buoy
   - Record task completion time and detected indicator color

Communications / reporting:
- Report:
    - Detected color of the indicator buoy
    - Total time to complete the task

Connections:
- Called by: TaskMaster.py (imports run_task3)
- Calls/Imports:
  - potential_fields_planner.py -> PotentialFieldsPlanner
  - CreateMap.entities -> EntityList (used indirectly via passed-in entities)
- Standard libs: numpy, os, sys, time, enum
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple
import time
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]


@dataclass
class GoalPlan:
    """Individual goal with completion tracking"""
    goal_id: str
    position: Vec2
    completed: bool = False


@dataclass
class DetectedEntity:
    entity_id: int
    entity_type: str
    position: Vec2


class TaskStatus(Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


class Phase(Enum):
    ENTRY = "ENTRY"
    SEARCH_YELLOW = "SEARCH_YELLOW"
    LOOP_AROUND = "LOOP_AROUND"
    EXIT = "EXIT"


def segments_intersect(p1: Vec2, p2: Vec2, q1: Vec2, q2: Vec2) -> bool:
    """Check if two line segments intersect"""
    def _orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def _on_segment(a, b, c):
        return (min(a[0], b[0]) <= c[0] <= max(a[0], b[0]) and
                min(a[1], b[1]) <= c[1] <= max(a[1], b[1]))

    o1 = _orient(p1, p2, q1)
    o2 = _orient(p1, p2, q2)
    o3 = _orient(q1, q2, p1)
    o4 = _orient(q1, q2, p2)

    if (o1 * o2 < 0) and (o3 * o4 < 0):
        return True

    if abs(o1) < 1e-9 and _on_segment(p1, p2, q1): return True
    if abs(o2) < 1e-9 and _on_segment(p1, p2, q2): return True
    if abs(o3) < 1e-9 and _on_segment(q1, q2, p1): return True
    if abs(o4) < 1e-9 and _on_segment(q1, q2, p2): return True
    return False


class Task3Manager:
    def __init__(self, entities, map_bounds: Vec2, start_pose: Pose):
        self.entities = entities
        self.map_bounds = (float(map_bounds[0]), float(map_bounds[1]))
        self.pose: Pose = start_pose
        self.prev_pos: Optional[Vec2] = None

        self.phase = Phase.ENTRY

        # Gate tracking
        self.entrance_gate: Optional[Tuple[Vec2, Vec2]] = None
        self.gate_center: Optional[Vec2] = None

        # Crossing tracking
        self.entry_crossed = False
        self.exit_crossed = False

        # Task entities
        self.indicator_color: Optional[str] = None  # "red" or "green"
        self.yellow_pos: Optional[Vec2] = None

        # NEW: lock a loop waypoint so it doesn't move every tick
        self.loop_goal: Optional[Vec2] = None

        # Timing
        self.start_time: Optional[float] = None
        self.finish_time: Optional[float] = None

        # Goal output
        self.goal_queue: List[Vec2] = []

        # Planning output
        self.current_path: List[Vec2] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float) -> None:
        # keep previous position for gate-crossing segment test
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        else:
            self.prev_pos = (self.pose[0], self.pose[1])

        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        # Prefer dedup if available, otherwise fallback to add()
        if hasattr(self.entities, "add_or_update"):
            self.entities.add_or_update(det.entity_type, det.position,
                                        name=f"{det.entity_type}_{det.entity_id}")
        else:
            self.entities.add(det.entity_type, det.position,
                              name=f"{det.entity_type}_{det.entity_id}")

        if det.entity_type in ("red_indicator", "green_indicator") and self.indicator_color is None:
            self.indicator_color = "red" if det.entity_type == "red_indicator" else "green"

        if det.entity_type == "yellow_buoy":
            self.yellow_pos = det.position

    def _write_goals(self, goals: List[Vec2]) -> None:
        self.entities.clear_goals()
        for i, g in enumerate(goals, start=1):
            self.entities.add("goal", g, name=f"goal_wp{i}")
        self.goal_queue = goals[:]

    def _ensure_gate(self) -> None:
        if self.entrance_gate is not None:
            return
        gates = self.entities.get_gates()
        if not gates:
            return
        self.entrance_gate = gates[0]
        red, green = self.entrance_gate
        self.gate_center = ((red[0] + green[0]) / 2.0, (red[1] + green[1]) / 2.0)

    def _check_gate_cross(self) -> None:
        if self.prev_pos is None or self.entrance_gate is None:
            return
        red, green = self.entrance_gate
        if segments_intersect(self.prev_pos, (self.pose[0], self.pose[1]), red, green):
            if not self.entry_crossed:
                self.entry_crossed = True
                self.start_time = time.time()
            elif self.phase == Phase.EXIT and not self.exit_crossed:
                self.exit_crossed = True
                self.finish_time = time.time()

    def _compute_locked_loop_goal(self) -> Vec2:
        """Compute a single tangential point around the yellow buoy (LOCKED)."""
        yb = np.array(self.yellow_pos, dtype=float)
        boat = np.array(self.pose[:2], dtype=float)

        r = boat - yb
        if np.linalg.norm(r) < 1e-6:
            r = np.array([1.0, 0.0], dtype=float)
        r = r / (np.linalg.norm(r) + 1e-9)

        # Tangent depends on direction:
        # (Keep your mapping: red -> CCW, green -> CW)
        if self.indicator_color == "red":      # CCW
            t = np.array([-r[1], r[0]], dtype=float)
        else:                                  # CW
            t = np.array([r[1], -r[0]], dtype=float)

        beyond = yb + t * 20.0
        beyond[0] = float(np.clip(beyond[0], 0, self.map_bounds[0]))
        beyond[1] = float(np.clip(beyond[1], 0, self.map_bounds[1]))
        return (float(beyond[0]), float(beyond[1]))

    def tick(self) -> None:
        self._ensure_gate()
        self._check_gate_cross()

        # No gate yet: move forward a bit
        if self.entrance_gate is None or self.gate_center is None:
            x, y, hdg = self.pose
            g = (min(self.map_bounds[0], x + 40.0), min(self.map_bounds[1], y + 40.0))
            self._write_goals([g])
            return

        if self.phase == Phase.ENTRY:
            self._write_goals([self.gate_center])
            if self.entry_crossed:
                self.phase = Phase.SEARCH_YELLOW
                self.loop_goal = None  # reset just in case

        elif self.phase == Phase.SEARCH_YELLOW:
            x, y, hdg = self.pose
            left = np.array([-np.sin(hdg), np.cos(hdg)], dtype=float)
            right = -left
            bias = right if self.indicator_color == "red" else left  # your mapping
            target = (
                np.array([x, y], dtype=float)
                + bias * 25.0
                + np.array([np.cos(hdg), np.sin(hdg)], dtype=float) * 25.0
            )
            target[0] = float(np.clip(target[0], 0, self.map_bounds[0]))
            target[1] = float(np.clip(target[1], 0, self.map_bounds[1]))
            self._write_goals([tuple(target)])

            if self.yellow_pos is not None and self.indicator_color is not None:
                self.phase = Phase.LOOP_AROUND
                self.loop_goal = None  # IMPORTANT: recompute ONCE on entry

        elif self.phase == Phase.LOOP_AROUND:
            # IMPORTANT FIX: lock the loop goal once so it doesn't move each tick
            if self.loop_goal is None:
                self.loop_goal = self._compute_locked_loop_goal()

            self._write_goals([self.loop_goal])

            boat = np.array(self.pose[:2], dtype=float)
            if float(np.linalg.norm(boat - np.array(self.loop_goal))) < 4.0:
                self.phase = Phase.EXIT

        elif self.phase == Phase.EXIT:
            self._write_goals([self.gate_center])
            return

    def plan(self, planner: PotentialFieldsPlanner) -> None:
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return

        start = (self.pose[0], self.pose[1])
        obstacles = list(self.entities.get_obstacles())
        gates = self.entities.get_gates()

        result = planner.plan_multi_goal_path(
            start,
            self.goal_queue[:2],
            obstacles,
            gates=gates,
            map_bounds=self.map_bounds
        )
        self.current_path = list(map(tuple, result["path"]))
        self.current_velocities = result["velocities"]
        self.current_speeds = result["speeds"]

    def done(self) -> bool:
        return bool(self.exit_crossed)


def run_task3(
    entities,
    map_bounds,
    *,
    get_pose=None,
    get_detections=None,
    sleep_dt: float = 0.1,
    max_iters: int = 4000,
):
    start = entities.get_start()
    mgr = Task3Manager(entities, map_bounds, (float(start[0]), float(start[1]), 0.0))
    planner = PotentialFieldsPlanner(resolution=0.5)

    if get_pose is None:
        mgr.tick()
        try:
            mgr.plan(planner)
        except Exception as e:
            print(f"[TASK3] Planning warning: {e}")
        status = TaskStatus.SUCCESS if mgr.done() else TaskStatus.RUNNING
        return {
            "status": status.value,
            "entities_with_goals": mgr.entities,
            "phase": mgr.phase.value,
            "indicator_color": mgr.indicator_color,
            "time_s": None if mgr.finish_time is None or mgr.start_time is None else (mgr.finish_time - mgr.start_time),
            "path": np.array(mgr.current_path, dtype=float) if mgr.current_path else np.zeros((0, 2)),
            "velocities": mgr.current_velocities,
            "speeds": mgr.current_speeds,
        }

    for _ in range(max_iters):
        x, y, hdg = get_pose()
        mgr.update_pose(x, y, hdg)

        if get_detections is not None:
            for det in get_detections():
                mgr.add_detected(det)

        mgr.tick()
        try:
            mgr.plan(planner)
        except Exception as e:
            print(f"[TASK3] Planning warning: {e}")

        if mgr.done():
            return {
                "status": TaskStatus.SUCCESS.value,
                "entities_with_goals": mgr.entities,
                "phase": mgr.phase.value,
                "indicator_color": mgr.indicator_color,
                "time_s": None if mgr.finish_time is None or mgr.start_time is None else (mgr.finish_time - mgr.start_time),
                "path": np.array(mgr.current_path, dtype=float) if mgr.current_path else np.zeros((0, 2)),
                "velocities": mgr.current_velocities,
                "speeds": mgr.current_speeds,
            }

        time.sleep(sleep_dt)

    return {
        "status": TaskStatus.FAILURE.value,
        "reason": "max_iters reached",
        "entities_with_goals": mgr.entities,
        "phase": mgr.phase.value,
        "indicator_color": mgr.indicator_color,
        "time_s": None if mgr.finish_time is None or mgr.start_time is None else (mgr.finish_time - mgr.start_time),
        "path": np.array(mgr.current_path, dtype=float) if mgr.current_path else np.zeros((0, 2)),
        "velocities": mgr.current_velocities,
        "speeds": mgr.current_speeds,
    }