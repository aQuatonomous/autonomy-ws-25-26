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
      along this direction, up to a maximum of 100 ft
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
import time
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]  # x, y, heading (rad)


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


# -------------------------
# geometry: segment intersection
# -------------------------

def _orient(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    return float((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))

def _on_segment(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
    return (
        min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9
        and min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9
    )

def segments_intersect(p1: Vec2, p2: Vec2, q1: Vec2, q2: Vec2) -> bool:
    a = np.array(p1, dtype=float)
    b = np.array(p2, dtype=float)
    c = np.array(q1, dtype=float)
    d = np.array(q2, dtype=float)

    o1 = _orient(a, b, c)
    o2 = _orient(a, b, d)
    o3 = _orient(c, d, a)
    o4 = _orient(c, d, b)

    if (o1 * o2 < 0.0) and (o3 * o4 < 0.0):
        return True

    if abs(o1) < 1e-9 and _on_segment(a, b, c): return True
    if abs(o2) < 1e-9 and _on_segment(a, b, d): return True
    if abs(o3) < 1e-9 and _on_segment(c, d, a): return True
    if abs(o4) < 1e-9 and _on_segment(c, d, b): return True
    return False


# -------------------------
# Task 1 manager
# -------------------------

class Task1Manager:
    def __init__(self, entities, map_bounds: Vec2, start_pose: Pose):
        self.entities = entities
        self.map_bounds = (float(map_bounds[0]), float(map_bounds[1]))

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
        self.goal_queue: List[Vec2] = []
        self.current_path: List[Vec2] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float) -> None:
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        else:
            self.prev_pos = (self.pose[0], self.pose[1])
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        """Add detected entity with deduplication"""
        self.entities.add_or_update(det.entity_type, det.position, 
                                   name=f"{det.entity_type}_{det.entity_id}")

    def _lock_gates_once(self) -> None:
        """Lock the 2 gates once discovered to prevent flicker/order changes"""
        if self.gates_locked:
            return
            
        gates = self.entities.get_gates()
        if len(gates) >= 2:  # Task1 expects exactly 2 gates
            self.locked_gates = gates[:2]  # Take first 2 gates
            self.gates_locked = True
            
            # Create persistent goal plan for the 2 gate centers
            self.goal_plan = []
            centers = [((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0) for (r, g) in self.locked_gates]
            
            # Order by forward direction
            fwd = self._forward_dir(centers)
            s = np.array(self.start_pos, dtype=float)
            scored = []
            for i, c in enumerate(centers):
                v = np.array(c, dtype=float) - s
                proj = float(v.dot(fwd))
                scored.append((proj, i, c))
            scored.sort(key=lambda t: t[0])
            
            for i, (_, _, center) in enumerate(scored):
                goal = GoalPlan(
                    goal_id=f"gate_{i+1}",
                    position=center,
                    completed=False
                )
                self.goal_plan.append(goal)

    def publish_goals(self) -> None:
        """Publish only current active goals to EntityList as goal_wp*"""
        self.entities.clear_goals()
        
        active_goals = []
        for goal in self.goal_plan:
            if not goal.completed:
                active_goals.append(goal.position)
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

        # else start -> nearest center
        if centers:
            s = np.array(self.start_pos, dtype=float)
            c = np.array(centers, dtype=float)
            idx = int(np.argmin(np.linalg.norm(c - s[None, :], axis=1)))
            u = c[idx] - s
            n = np.linalg.norm(u)
            if n > 1e-6:
                return u / n

        # else default north
        return np.array([0.0, 1.0], dtype=float)

    def _compute_gate_centers_ordered(self) -> None:
        gates = self.entities.get_gates()
        if len(gates) == 0:
            self.ordered_gates = []
            self.gate_centers = []
            return

        # update last_gate_normal from ANY seen gate
        red, green = gates[-1]
        gate_vec = np.array([green[0] - red[0], green[1] - red[1]], dtype=float)
        if np.linalg.norm(gate_vec) > 1e-6:
            n = np.array([-gate_vec[1], gate_vec[0]], dtype=float)
            self.last_gate_normal = n / (np.linalg.norm(n) + 1e-9)

        # center list
        centers = [((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0) for (r, g) in gates]
        fwd = self._forward_dir(centers)
        s = np.array(self.start_pos, dtype=float)

        # sort by projection along fwd (then distance)
        scored = []
        for (gate, c) in zip(gates, centers):
            v = np.array(c, dtype=float) - s
            proj = float(v.dot(fwd))
            dist = float(np.linalg.norm(v))
            scored.append((proj, dist, gate, c))
        scored.sort(key=lambda t: (t[0], t[1]))

        # ONLY 2 gates for task 1
        scored = scored[:2]
        self.ordered_gates = [t[2] for t in scored]
        self.gate_centers = [t[3] for t in scored]

        # if next_gate_index is out of range (e.g., reset), clamp
        self.next_gate_index = min(self.next_gate_index, max(0, len(self.gate_centers) - 1))

    def _fallback_goal(self) -> Vec2:
        # perpendicular to most recently detected gate line, forward 100ft
        x, y, hdg = self.pose
        if self.last_gate_normal is None:
            # if no history, move along heading
            d = np.array([np.cos(hdg), np.sin(hdg)], dtype=float)
            d = d / (np.linalg.norm(d) + 1e-9)
        else:
            d = self.last_gate_normal

        step = 100.0
        p = np.array([x, y], dtype=float) + d * step
        p[0] = float(np.clip(p[0], 0, self.map_bounds[0]))
        p[1] = float(np.clip(p[1], 0, self.map_bounds[1]))
        return (float(p[0]), float(p[1]))

    def _write_goals(self, goals: List[Vec2]) -> None:
        self.entities.clear_goals()
        for i, g in enumerate(goals, start=1):
            self.entities.add("goal", g, name=f"goal_wp{i}")
        self.goal_queue = list(goals)

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
        
        # If no locked gates yet, use fallback
        if not self.gates_locked:
            fallback_goal = self._fallback_goal()
            self.entities.clear_goals()
            self.entities.add("goal", fallback_goal, name="goal_wp1")
            self.goal_queue = [fallback_goal]
        else:
            # Publish current active goals
            self.publish_goals()
    
    def done(self) -> bool:
        """Return True if all goals completed (SUCCESS)"""
        return all(goal.completed for goal in self.goal_plan) and len(self.goal_plan) > 0

    def plan(self, planner: PotentialFieldsPlanner, max_goals: int = 1) -> None:
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return

        start = (self.pose[0], self.pose[1])
        obstacles = list(self.entities.get_obstacles())
        gates = self.locked_gates if self.gates_locked else self.entities.get_gates()
        goals = self.goal_queue[:max_goals]

        out = planner.plan_multi_goal_path(start, goals, obstacles, gates=gates, map_bounds=self.map_bounds)
        self.current_path = list(map(tuple, out["path"]))
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]


def run_task1(
    entities,
    map_bounds: Vec2,
    *,
    one_shot: bool = True,
    get_pose: Optional[Callable[[], Pose]] = None,
    get_detections: Optional[Callable[[], List[DetectedEntity]]] = None,
    plan: bool = True,
    sleep_dt: float = 0.1,
    max_iters: int = 3000,
) -> Dict:
    """
    one_shot=True:
      - run one tick and return RUNNING/SUCCESS
    loop mode (one_shot=False):
      - requires get_pose, runs until SUCCESS or max_iters
    """
    start = entities.get_start()
    mgr = Task1Manager(entities, map_bounds, (float(start[0]), float(start[1]), 0.0))
    planner = PotentialFieldsPlanner(resolution=0.5)

    def step_once():
        if get_pose is not None:
            x, y, hdg = get_pose()
            mgr.update_pose(x, y, hdg)
        if get_detections is not None:
            for det in get_detections():
                mgr.add_detected(det)
        mgr.tick()
        if plan:
            mgr.plan(planner, max_goals=1)

    # one-shot
    if one_shot:
        step_once()
        return {
            "status": TaskStatus.SUCCESS.value if mgr.done() else TaskStatus.RUNNING.value,
            "goals": list(mgr.goal_queue),
            "next_gate_index": mgr.next_gate_index,
            "path": np.array(mgr.current_path, dtype=float) if mgr.current_path else np.zeros((0, 2)),
            "velocities": mgr.current_velocities,
            "speeds": mgr.current_speeds,
            "entities": mgr.entities,
        }

    # loop mode
    if get_pose is None:
        return {"status": TaskStatus.FAILURE.value, "reason": "loop mode requires get_pose"}

    for _ in range(max_iters):
        step_once()
        if mgr.done():
            return {
                "status": TaskStatus.SUCCESS.value,
                "goals": list(mgr.goal_queue),
                "next_gate_index": mgr.next_gate_index,
                "path": np.array(mgr.current_path, dtype=float) if mgr.current_path else np.zeros((0, 2)),
                "velocities": mgr.current_velocities,
                "speeds": mgr.current_speeds,
                "entities": mgr.entities,
            }
        time.sleep(sleep_dt)

    return {"status": TaskStatus.FAILURE.value, "reason": "max_iters reached"}
