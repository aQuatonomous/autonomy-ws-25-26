#!/usr/bin/env python3
"""
Global Planner Task 1 (Only 2 gates)

What this module does (Task 1):
- Input: EntityList (types + positions) from JSON/perception
- Output: Updated EntityList with ORDERED goal waypoints: goal_wp1, goal_wp2, ...

Key rules implemented:
1) Robust goal ordering:
   - Build gate centers from paired (red_buoy, green_buoy)
   - Compute a "forward direction" vector:
       a) from boat heading if available, else
       b) from start -> nearest gate center
   - Sort gate centers by projection onto that forward direction

2) Gate completion:
   - A gate is "completed" only when the boat motion segment
     (prev_pos -> current_pos) intersects the gate segment (red -> green)

3) Fallback (no gate detected):
    - If no next valid (red_buoy, green_buoy) gate pair is detected:
    - Define the forward direction as the unit vector perpendicular to the
      most recently detected gate line (red_buoy → green_buoy)
    - Generate a temporary forward waypoint by projecting straight ahead
      along this direction, up to a maximum of 100 m
    - Exit fallback immediately once the next gate is detected

One-shot mode:
- Compute and write goals once, optionally compute a path once, return RUNNING/SUCCESS.

Loop mode:
- Repeatedly update pose/perception, update goals, optionally plan,
  return SUCCESS once both gates are crossed in order.
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner

from Global.goal_utils import nudge_goal_away_from_obstacles, segments_intersect

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]  # x, y, heading (rad)


@dataclass
class GoalPlan:
    """Individual goal with completion tracking"""
    goal_id: str
    position: Vec3  # (x, y, heading_rad); Task1 uses heading=0
    completed: bool = False


from Global.types import DetectedEntity, Vec3


class TaskStatus(Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


# -------------------------
# Task 1 manager
# -------------------------

class Task1Manager:
    def __init__(self, entities, map_bounds: Optional[Vec2], start_pose: Pose):
        self.entities = entities
        self.map_bounds = (float(map_bounds[0]), float(map_bounds[1])) if map_bounds else None

        self.start_pos: Vec2 = (float(start_pose[0]), float(start_pose[1]))
        self.pose: Pose = start_pose
        self.prev_pos: Optional[Vec2] = None

        # Persistent goal plan with completion tracking
        self.goal_plan: List[GoalPlan] = []
        self.locked_gates: List[Tuple[Vec2, Vec2]] = []  # Gates locked once discovered
        self.gates_locked: bool = False
        
        # completion tracking
        self.next_gate_index: int = 0
        self.last_gate_normal: Optional[np.ndarray] = None

        # outputs
        self.goal_queue: List[Vec3] = []
        self.current_path: List[Vec3] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float) -> None:
        # prev_pos is updated at end of tick() so it is always "previous tick's position". Here only init if needed.
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        """Add detected entity by id (same id = same entity). Skip add_or_update if already in list (e.g. from fusion)."""
        already = any(getattr(e, "entity_id", None) == det.entity_id for e in self.entities.entities)
        if not already:
            self.entities.add_or_update(det.entity_type, det.position, det.entity_id,
                                        name=f"{det.entity_type}_{det.entity_id}")

    def _lock_gates_once(self) -> None:
        """
        Lock gates as we see them: first gate when we see >= 1 (go through it), then add second gate
        when we see it (go forward until we see it, then through it). Done when through both.
        Gates are filtered perpendicular to boat (left/right of boat).
        """
        gates = self.entities.get_gates(boat_heading_rad=self.pose[2])
        if not gates:
            return

        centers = [((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0, 0.0) for (r, g) in gates]
        fwd = self._forward_dir(centers)
        s_arr = np.array(getattr(self.entities, "get_start", lambda: self.start_pos)(), dtype=float)[:2]
        scored = [(float(np.array(c[:2], dtype=float).dot(fwd) - s_arr.dot(fwd)), i, c) for i, c in enumerate(centers)]
        scored.sort(key=lambda t: t[0])  # order by distance along forward from start

        if not self.gates_locked:
            # Lock first gate as soon as we see at least one
            self.locked_gates = [gates[scored[0][1]]]
            self.gates_locked = True
            self.goal_plan = [
                GoalPlan(goal_id="gate_1", position=scored[0][2], completed=False)
            ]
            return

        if len(self.locked_gates) == 1 and len(gates) >= 2:
            # Add second gate when we see it (ahead in forward order)
            second_idx = scored[1][1]
            second_gate = gates[second_idx]
            # Avoid duplicating the same gate
            if second_gate != self.locked_gates[0]:
                self.locked_gates.append(second_gate)
                self.goal_plan.append(
                    GoalPlan(goal_id="gate_2", position=scored[1][2], completed=False)
                )

    def publish_goals(self) -> None:
        """Publish only current active goals to EntityList as goal_wp*; nudge away from obstacles."""
        self.entities.clear_goals()
        obstacles = self._obstacles()
        from_pt = (self.pose[0], self.pose[1], self.pose[2])

        active_goals = []
        for goal in self.goal_plan:
            if not goal.completed:
                pos = nudge_goal_away_from_obstacles(goal.position, obstacles, from_pt)
                active_goals.append(pos)
                # Only publish the next goal, not all remaining goals
                break

        for i, pos in enumerate(active_goals, start=1):
            self.entities.add("goal", pos, name=f"goal_wp{i}")

        self.goal_queue = active_goals

    def _forward_dir(self, centers: List[Vec2]) -> np.ndarray:
        # heading preferred
        hdg = float(self.pose[2])
        v = np.array([np.cos(hdg), np.sin(hdg)], dtype=float)
        if np.linalg.norm(v) > 1e-6:
            return v / (np.linalg.norm(v) + 1e-9)

        # else start -> nearest center (use mission start if available)
        if centers:
            s = np.array(getattr(self.entities, "get_start", lambda: self.start_pos)(), dtype=float)[:2]
            c = np.array([[p[0], p[1]] for p in centers], dtype=float)
            idx = int(np.argmin(np.linalg.norm(c - s[None, :], axis=1)))
            u = c[idx] - s
            n = np.linalg.norm(u)
            if n > 1e-6:
                return u / n

        # else default north
        return np.array([0.0, 1.0], dtype=float)

    def _obstacles(self) -> List[Tuple[float, ...]]:
        """Obstacles = buoys + no-go (gate walls + map bounds when map_bounds set). No-go uses boat heading so it matches locked gates."""
        return list(self.entities.get_obstacles()) + list(
            self.entities.get_no_go_obstacle_points(
                map_bounds=self.map_bounds,
                boat_heading_rad=self.pose[2],
            )
        )

    def _fallback_goal(self) -> Vec2:
        # perpendicular to most recently detected gate line, forward 100 m; nudge away from obstacles
        x, y, hdg = self.pose
        if self.last_gate_normal is None:
            # if no history, move along heading
            d = np.array([np.cos(hdg), np.sin(hdg)], dtype=float)
            d = d / (np.linalg.norm(d) + 1e-9)
        else:
            d = self.last_gate_normal

        step = 100.0
        p = np.array([x, y], dtype=float) + d * step
        if self.map_bounds is not None:
            p[0] = float(np.clip(p[0], 0, self.map_bounds[0]))
            p[1] = float(np.clip(p[1], 0, self.map_bounds[1]))
        desired = (float(p[0]), float(p[1]), 0.0)
        return nudge_goal_away_from_obstacles(desired, self._obstacles(), (x, y, hdg))

    def _check_goal_completion(self) -> None:
        """Check if current goal was reached and mark it complete"""
        if self.prev_pos is None or not self.goal_plan:
            return
            
        current_goal = None
        goal_index = -1
        for i, goal in enumerate(self.goal_plan):
            if not goal.completed:
                current_goal = goal
                goal_index = i
                break
                
        if current_goal is None:
            return
            
        # Check if we crossed the corresponding gate
        if goal_index < len(self.locked_gates):
            red, green = self.locked_gates[goal_index]
            cur = (self.pose[0], self.pose[1])
            if segments_intersect(self.prev_pos, cur, red, green):
                current_goal.completed = True
                self.next_gate_index = goal_index + 1

    def tick(self) -> None:
        """Main update loop - lock gates, check completion, publish active goals"""
        # Lock gates once we have enough
        self._lock_gates_once()
        
        # Check if current goal was completed
        self._check_goal_completion()
        
        # Update last_gate_normal from any detected gate (for fallback direction)
        gates = self.entities.get_gates(boat_heading_rad=self.pose[2])
        if gates:
            red, green = gates[0]
            along = np.array([green[0] - red[0], green[1] - red[1]], dtype=float)
            n = np.array([-along[1], along[0]], dtype=float)
            nnorm = np.linalg.norm(n) + 1e-9
            if nnorm > 1e-6:
                self.last_gate_normal = n / nnorm

        # If no locked gates yet, use fallback (go forward to find first gate)
        if not self.gates_locked:
            fallback_goal = self._fallback_goal()
            self.entities.clear_goals()
            self.entities.add("goal", fallback_goal, name="goal_wp1")
            self.goal_queue = [fallback_goal]
        else:
            self.publish_goals()
            # After gate 1, go forward until we see gate 2 (fallback keeps us moving)
            if not self.goal_queue and len(self.locked_gates) == 1:
                fallback_goal = self._fallback_goal()
                self.entities.clear_goals()
                self.entities.add("goal", fallback_goal, name="goal_wp1")
                self.goal_queue = [fallback_goal]

        # So next tick _check_goal_completion uses segment (this tick, next tick)
        self.prev_pos = (self.pose[0], self.pose[1])

    def done(self) -> bool:
        """Return True when we have gone through both gates (SUCCESS)."""
        return len(self.goal_plan) >= 2 and all(goal.completed for goal in self.goal_plan)

    def plan(self, planner: PotentialFieldsPlanner, max_goals: int = 1) -> None:
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return

        start = (self.pose[0], self.pose[1])
        obstacles = self._obstacles()
        gates = self.locked_gates if self.gates_locked else self.entities.get_gates()
        goals = self.goal_queue[:max_goals]

        goals_xy = [(g[0], g[1]) for g in goals]
        out = planner.plan_multi_goal_path(start, goals_xy, obstacles, gates=gates, map_bounds=self.map_bounds)
        self.current_path = [(float(p[0]), float(p[1]), 0.0) for p in out["path"]]
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]


def run_task1(
    entities,
    map_bounds: Optional[Vec2],
    *,
    get_pose: Optional[Callable[[], Pose]] = None,
    get_detections: Optional[Callable[[], List[DetectedEntity]]] = None,
    plan: bool = True,
) -> Dict:
    """
    Run one shot: one tick (update pose/detections, tick, optionally plan), then return RUNNING/SUCCESS.
    Caller repeats run_task1 until done (e.g. from a ROS timer).
    """
    start = entities.get_start()
    mgr = Task1Manager(entities, map_bounds, (float(start[0]), float(start[1]), 0.0))  # map_bounds can be None
    planner = PotentialFieldsPlanner(resolution=0.5)

    if get_pose is not None:
        x, y, hdg = get_pose()
        mgr.update_pose(x, y, hdg)
    if get_detections is not None:
        for det in get_detections():
            mgr.add_detected(det)
    mgr.tick()
    if plan:
        mgr.plan(planner, max_goals=1)

    return {
        "status": TaskStatus.SUCCESS.value if mgr.done() else TaskStatus.RUNNING.value,
        "goals": list(mgr.goal_queue),
        "next_gate_index": mgr.next_gate_index,
        "path": np.array(mgr.current_path, dtype=float) if mgr.current_path else np.zeros((0, 2)),
        "velocities": mgr.current_velocities,
        "speeds": mgr.current_speeds,
        "entities": mgr.entities,
    }

