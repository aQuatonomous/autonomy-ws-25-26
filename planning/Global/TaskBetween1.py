"""
ASV Challenge Task - [Insert Specific Task Name Here, e.g. "Gate Navigation with Obstacle Avoidance"]

What this module does:
- Input: EntityList (types + positions) from JSON/perception, current boat pose
- Output: Updated EntityList with ORDERED goal waypoints: goal_wp1, goal_wp2, ...

Key rules implemented:
1) [Primary strategy #1 - e.g. "Gate detection and ordering by forward projection"]
2) [Primary strategy #2 - e.g. "Obstacle-aware goal nudging and completion checking"] 
3) [Fallback behavior - e.g. "Forward projection when no gates detected"]
4) [Completion criteria - e.g. "Segment intersection for gate crossing validation"]

One-shot mode:
- Compute and write goals once, optionally compute path once, return RUNNING/SUCCESS.

Loop mode:
- Repeatedly update pose/perception, update goals, optionally plan, return SUCCESS when task complete.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner
from Global.goal_utils import nudge_goal_away_from_obstacles, segments_intersect
from Global.types import DetectedEntity, Vec3

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]  # x, y, heading (rad)

@dataclass
class GoalPlan:
    """Individual goal with completion tracking"""
    goal_id: str
    position: Vec3  # (x, y, heading_rad)
    completed: bool = False

class TaskStatus(Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"

# -------------------------
# Main Task Manager Class
# -------------------------  
class ASVTaskManager:
    def __init__(self, entities, map_bounds, start_pose: Pose):
        self.entities = entities
        self.map_bounds = tuple(float(v) for v in map_bounds) if map_bounds else None
        
        self.start_pos: Vec2 = (float(start_pose[0]), float(start_pose[1]))
        self.pose: Pose = start_pose
        self.prev_pos: Optional[Vec2] = None
        
        # Persistent state tracking
        self.goal_plan: List[GoalPlan] = []
        self.next_goal_index: int = 0
        self.task_phase: str = "planning"  # Customize phases as needed
        
        # Outputs
        self.goal_queue: List[Vec3] = []
        self.current_path: List[Vec3] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float) -> None:
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        already = any(getattr(e, "entity_id", None) == det.entity_id for e in self.entities.entities)
        if not already:
            self.entities.add_or_update(det.entity_type, det.position, det.entity_id,
                                      name=f"{det.entity_type}_{det.entity_id}")

    def _get_relevant_entities(self) -> List[Tuple]:
        """
        Extract task-specific entities from EntityList.
        Assume nodes have required info (gates, obstacles, targets, etc.)
        """
        boat_xy = (self.pose[0], self.pose[1])
        # Customize based on task: gates, targets, obstacles, etc.
        return self.entities.get_gates(boat_heading_rad=self.pose[2], boat_pos=boat_xy)

    def _build_goal_plan(self) -> None:
        """Construct ordered goal sequence based on current perception."""
        entities = self._get_relevant_entities()
        
        if not entities:
            return
            
        # Task-specific goal generation logic here
        # Example: sort gates by forward projection
        fwd_dir = self._forward_dir()
        boat_x, boat_y = self.pose[0], self.pose[1]
        
        def projection_score(entity):
            cx, cy = entity[0][0], entity[0][1]  # Assume center available in node
            return (cx - boat_x) * fwd_dir[0] + (cy - boat_y) * fwd_dir[1]
        
        sorted_entities = sorted(entities, key=projection_score)
        
        self.goal_plan = []
        for i, entity in enumerate(sorted_entities[:3]):  # Limit goals
            # Assume node provides center position
            center = (entity[0][0], entity[0][1], 0.0)
            self.goal_plan.append(GoalPlan(f"goal_{i+1}", center))

    def _forward_dir(self) -> np.ndarray:
        """Compute forward direction from boat heading."""
        hdg = float(self.pose[2])
        return np.array([np.cos(hdg), np.sin(hdg)], dtype=float)

    def _obstacles(self) -> List[Tuple[float, ...]]:
        """Get current obstacles excluding task-relevant entities."""
        gate_ids = self.entities.get_gate_entity_ids(
            boat_heading_rad=self.pose[2],
            boat_pos=(self.pose[0], self.pose[1]),
        )
        return (list(self.entities.get_obstacles(exclude_entity_ids=gate_ids)) + 
                list(self.entities.get_no_go_obstacle_points(
                    map_bounds=self.map_bounds, boat_heading_rad=self.pose[2])))

    def publish_goals(self) -> None:
        """Publish next active goal waypoint(s)."""
        self.entities.clear_goals()
        obstacles = self._obstacles()
        from_pt = (self.pose[0], self.pose[1], self.pose[2])
        
        active_goals = []
        for goal in self.goal_plan[self.next_goal_index:]:
            if not goal.completed:
                nudged = nudge_goal_away_from_obstacles(goal.position, obstacles, from_pt)
                active_goals.append(nudged)
                break
        
        for i, pos in enumerate(active_goals, start=1):
            self.entities.add("goal", pos, name=f"goal_wp{i}")
        
        self.goal_queue = active_goals

    def _check_goal_completion(self) -> None:
        """Check if boat crossed current goal segment."""
        if self.prev_pos is None or not self.goal_plan:
            return
            
        cur = (self.pose[0], self.pose[1])
        if (abs(cur[0] - self.prev_pos[0]) < 1e-9 and 
            abs(cur[1] - self.prev_pos[1]) < 1e-9):
            return
            
        current_goal = self.goal_plan[self.next_goal_index]
        # Assume gate segment available in goal node data
        if hasattr(current_goal, 'segment') and segments_intersect(
            self.prev_pos, cur, current_goal.segment[0], current_goal.segment[1]):
            current_goal.completed = True
            self.next_goal_index += 1

    def _fallback_goal(self) -> Vec3:
        """Generate safe forward projection goal."""
        x, y, hdg = self.pose
        fwd = np.array([np.cos(hdg), np.sin(hdg)], dtype=float)
        target = np.array([x, y]) + fwd * 20.0
        return nudge_goal_away_from_obstacles(
            (float(target[0]), float(target[1]), 0.0), 
            self._obstacles(), self.pose)

    def tick(self) -> None:
        """Main update: perception → planning → goals → completion."""
        self._build_goal_plan()
        self._check_goal_completion()
        
        if not self.goal_plan or self.next_goal_index >= len(self.goal_plan):
            fallback = self._fallback_goal()
            self.entities.clear_goals()
            self.entities.add("goal", fallback, name="goal_wp1")
            self.goal_queue = [fallback]
        else:
            self.publish_goals()
        
        self.prev_pos = (self.pose[0], self.pose[1])

    def done(self) -> bool:
        """Task complete when all goals reached."""
        return (len(self.goal_plan) > 0 and 
                self.next_goal_index >= len(self.goal_plan) and
                all(goal.completed for goal in self.goal_plan))

    def plan(self, planner: PotentialFieldsPlanner, max_goals: int = 1) -> None:
        """Generate path to next goal(s) using potential fields."""
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return
            
        start = (self.pose[0], self.pose[1])
        obstacles = self._obstacles()
        goals_xy = [(g[0], g[1]) for g in self.goal_queue[:max_goals]]
        
        out = planner.plan_multi_goal_path(
            start, goals_xy, obstacles, 
            gates=[], map_bounds=self.map_bounds)
            
        self.current_path = [(float(p[0]), float(p[1]), 0.0) for p in out["path"]]
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]

def run_task(
    entities,
    map_bounds: Optional[Vec2],
    *,
    get_pose: Optional[Callable[[], Pose]] = None,
    get_detections: Optional[Callable[[], List[DetectedEntity]]] = None,
    plan: bool = True,
) -> Dict:
    """
    Execute one task update cycle.
    Caller loops until done() returns True.
    """
    start = entities.get_start()
    mgr = ASVTaskManager(entities, map_bounds, 
                        (float(start[0]), float(start[1]), 0.0))
    planner = PotentialFieldsPlanner(resolution=0.5)
    
    if get_pose is not None:
        x, y, hdg = get_pose()
        mgr.update_pose(x, y, hdg)
    if get_detections is not None:
        for det in get_detections():
            mgr.add_detected(det)
    
    mgr.tick()
    if plan:
        mgr.plan(planner)
    
    return {
        "status": TaskStatus.SUCCESS.value if mgr.done() else TaskStatus.RUNNING.value,
        "goals": list(mgr.goal_queue),
        "progress": mgr.next_goal_index,
        "path": np.array(mgr.current_path, dtype=float) if mgr.current_path else np.zeros((0, 2)),
        "velocities": mgr.current_velocities,
        "speeds": mgr.current_speeds,
        "entities": mgr.entities,
    }