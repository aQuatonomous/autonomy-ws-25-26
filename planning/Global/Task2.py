#!/usr/bin/env python3
"""Global Planner Task 2: Debris Clearance goal/phase manager + optional planning.

Location: src/Planner/Global/

What this module does (Task 2):
- Input: EntityList (types + positions) from JSON/perception
- Output: Updated EntityList with ORDERED task goals as goal_wp1, goal_wp2, ...

Task 2 behavior:
Phase A: TRANSIT_OUT
  - Calculate channel gate centers from red/green Buoy pairs.
  - If no gates known, set forward search goal.
  - Navigate gate centers in forward order while avoiding debris(channel).
  - Save gate centers for phase C return.
Phase B: DEBRIS_FIELD
  - Navigate into debris field straight until a green indicator (survivor) is seen.
  - Avoid hazards (red_indicator) + black buoys (black_buoy).
  - Report locations of all indicators + buoys seen.
  - For each survivor (green_indicator): "circle" (represented as circle waypoints).
    - Create 4 waypoints in a circle around survivor position that doesnt collide with obstacles.
  - Record/report positions (stored in returned dict; writing files is not done here).
Phase C: RETURN
  - Navigate back through the channel in reverse to start using waypoints saved in Phase A.

3) Fallback (no gate detected):
    - If no next valid (red_buoy, green_buoy) gate pair is detected for Part A ONLY:
    - Define the forward direction as the unit vector perpendicular to the
      most recently detected gate line (red_buoy → green_buoy)
    - Generate a temporary forward waypoint by projecting straight ahead
      along this direction, up to a maximum of 50 ft
    - Continue advancing and re-checking perception each update
    - Exit fallback immediately once the next gate is detected and
      resume normal gate ordering logic

Connections (kept):
- Called by: TaskMaster.py (imports run_task2)
- Calls/Imports:
  - potential_fields_planner.py -> PotentialFieldsPlanner
  - Global.entities -> EntityList (passed in)
- Standard libs: numpy, os, sys, time, enum, json
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple, Set
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
    TRANSIT_OUT = "TRANSIT_OUT"
    DEBRIS_FIELD = "DEBRIS_FIELD"
    RETURN = "RETURN"


class Report:
    """In-memory report for Phase B."""
    def __init__(self):
        self.green_indicators: Dict[str, Vec2] = {}
        self.red_indicators: Dict[str, Vec2] = {}
        self.black_buoys: Dict[str, Vec2] = {}

    def add(self, det: DetectedEntity):
        if det.entity_type == "green_indicator":
            self.green_indicators[f"green_{det.entity_id}"] = det.position
        elif det.entity_type == "red_indicator":
            self.red_indicators[f"red_{det.entity_id}"] = det.position
        elif det.entity_type == "black_buoy":
            self.black_buoys[f"black_{det.entity_id}"] = det.position

    def to_dict(self) -> Dict:
        def pack(d):
            return [{"name": k, "x": float(v[0]), "y": float(v[1])} for k, v in d.items()]
        return {
            "green_indicators": pack(self.green_indicators),
            "red_indicators": pack(self.red_indicators),
            "black_buoys": pack(self.black_buoys),
            "counts": {
                "green": len(self.green_indicators),
                "red": len(self.red_indicators),
                "black": len(self.black_buoys),
            },
        }


class Task2Manager:
    def __init__(self, entities, map_bounds: Vec2, start_pose: Pose):
        self.entities = entities
        self.map_bounds = (float(map_bounds[0]), float(map_bounds[1]))
        self.start_pos: Vec2 = (float(start_pose[0]), float(start_pose[1]))
        self.pose: Pose = start_pose

        # Persistent phase state
        self.phase = Phase.TRANSIT_OUT
        self.report = Report()

        # Persistent goal plan with completion tracking
        self.goal_plan: List[GoalPlan] = []
        
        # Phase A: channel centers (stored for Phase C return)
        self.channel_centers_out: List[Vec2] = []
        self.channel_locked: bool = False
        self.last_gate_normal: Optional[np.ndarray] = None

        # Phase B: survivor handling with persistent tracking
        self.handled_survivors: Set[str] = set()  # survivor IDs that were already circled
        self.current_survivor_plan: List[GoalPlan] = []  # 4-point circle plan for current survivor

        # Phase C: return waypoints (reverse channel + start)
        self.return_waypoints: List[Vec2] = []

        # goal output
        self.goal_queue: List[Vec2] = []

        # completion / tunables
        self.goal_reached_dist = 2.0
        self.debris_entry_y = 130.0  # simple map boundary for "entering debris field"
        self.mission_complete = False

        # optional path
        self.current_path: List[Vec2] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float):
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity):
        """Add detected entity with deduplication and reporting"""
        self.entities.add_or_update(det.entity_type, det.position, 
                                   name=f"{det.entity_type}_{det.entity_id}")
        # Always add to report for Phase B
        self.report.add(det)

    def publish_goals(self) -> None:
        """Publish only current active goals to EntityList as goal_wp*"""
        self.entities.clear_goals()
        
        active_goals = []
        for goal in self.goal_plan:
            if not goal.completed:
                active_goals.append(goal.position)
                # Publish a few goals ahead for better planning
                if len(active_goals) >= 2:
                    break
        
        for i, pos in enumerate(active_goals, start=1):
            self.entities.add("goal", pos, name=f"goal_wp{i}")
        
        self.goal_queue = active_goals

    def _lock_channel_centers(self) -> None:
        """Lock channel centers for Phase A and later return in Phase C"""
        if self.channel_locked:
            return
            
        centers = self._gate_centers_sorted()
        if len(centers) >= 2:  # Need at least 2 gates for meaningful channel
            self.channel_centers_out = centers
            self.channel_locked = True
            
            # Create goal plan for Phase A (outbound transit)
            self.goal_plan = []
            for i, center in enumerate(self.channel_centers_out):
                goal = GoalPlan(
                    goal_id=f"channel_out_{i+1}",
                    position=center,
                    completed=False
                )
                self.goal_plan.append(goal)

    def _check_goal_completion(self) -> None:
        """Check if current goal was reached and mark it complete"""
        if not self.goal_plan:
            return
            
        current_goal = None
        for goal in self.goal_plan:
            if not goal.completed:
                current_goal = goal
                break
                
        if current_goal is None:
            return
            
        # Check if we're close enough to the goal
        pos = (self.pose[0], self.pose[1])
        dist = self._distance(pos, current_goal.position)
        if dist <= self.goal_reached_dist:
            current_goal.completed = True

    def _create_survivor_circle_plan(self, survivor_pos: Vec2, survivor_id: str) -> None:
        """Create 4-point circle goals around a survivor"""
        if survivor_id in self.handled_survivors:
            return
            
        circle_goals = []
        cx, cy = survivor_pos
        radius = 6.0
        
        for i in range(4):
            angle = 2.0 * np.pi * (i / 4.0)
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            
            # Clamp to map bounds
            x = float(np.clip(x, 0, self.map_bounds[0]))
            y = float(np.clip(y, 0, self.map_bounds[1]))
            
            goal = GoalPlan(
                goal_id=f"circle_{survivor_id}_{i+1}",
                position=(x, y),
                completed=False
            )
            circle_goals.append(goal)
        
        self.current_survivor_plan = circle_goals
        self.handled_survivors.add(survivor_id)

    def _transition_to_debris_field(self) -> None:
        """Transition from Phase A to Phase B"""
        if self.phase != Phase.TRANSIT_OUT:
            return
            
        # Check if we've completed channel transit or reached debris field
        all_channel_complete = all(goal.completed for goal in self.goal_plan)
        reached_debris_y = self.pose[1] >= self.debris_entry_y
        
        if all_channel_complete or reached_debris_y:
            self.phase = Phase.DEBRIS_FIELD
            self.goal_plan = []  # Clear channel goals
            
            # Look for survivors to start circling
            survivors = self.entities.get_positions_by_type("green_indicator")
            if survivors:
                # Circle the first unhandled survivor
                for i, surv_pos in enumerate(survivors):
                    surv_id = f"survivor_{i}"
                    if surv_id not in self.handled_survivors:
                        self._create_survivor_circle_plan(surv_pos, surv_id)
                        self.goal_plan = self.current_survivor_plan
                        break

    def _transition_to_return(self) -> None:
        """Transition from Phase B to Phase C"""
        if self.phase != Phase.DEBRIS_FIELD:
            return
            
        # Check if current survivor circle is complete
        circle_complete = all(goal.completed for goal in self.current_survivor_plan)
        
        # Look for more unhandled survivors
        survivors = self.entities.get_positions_by_type("green_indicator")
        more_survivors = False
        for i, _ in enumerate(survivors):
            surv_id = f"survivor_{i}"
            if surv_id not in self.handled_survivors:
                more_survivors = True
                break
        
        if circle_complete and not more_survivors:
            # No more survivors, transition to return
            self.phase = Phase.RETURN
            self._create_return_plan()
        elif circle_complete and more_survivors:
            # More survivors to handle
            for i, surv_pos in enumerate(survivors):
                surv_id = f"survivor_{i}"
                if surv_id not in self.handled_survivors:
                    self._create_survivor_circle_plan(surv_pos, surv_id)
                    self.goal_plan = self.current_survivor_plan
                    break

    def _create_return_plan(self) -> None:
        """Create return plan using saved channel centers in reverse + start"""
        self.return_waypoints = list(reversed(self.channel_centers_out)) + [self.start_pos]
        
        self.goal_plan = []
        for i, waypoint in enumerate(self.return_waypoints):
            goal = GoalPlan(
                goal_id=f"return_{i+1}",
                position=waypoint,
                completed=False
            )
            self.goal_plan.append(goal)

    def tick(self) -> None:
        """Main update loop - manage phases, lock channels, check completion, publish goals"""
        
        if self.phase == Phase.TRANSIT_OUT:
            # Lock channel centers once discovered
            self._lock_channel_centers()
            
            # Check goal completion
            self._check_goal_completion()
            
            # Check for transition to debris field
            self._transition_to_debris_field()
            
            # If no locked channel yet, use fallback
            if not self.channel_locked:
                fallback_goal = self._fallback_goal_A()
                self.entities.clear_goals()
                self.entities.add("goal", fallback_goal, name="goal_wp1")
                self.goal_queue = [fallback_goal]
            else:
                self.publish_goals()
                
        elif self.phase == Phase.DEBRIS_FIELD:
            # Check goal completion
            self._check_goal_completion()
            
            # Check for phase transition
            self._transition_to_return()
            
            # Publish active goals
            self.publish_goals()
            
        elif self.phase == Phase.RETURN:
            # Check goal completion
            self._check_goal_completion()
            
            # Check if mission complete
            if all(goal.completed for goal in self.goal_plan):
                self.mission_complete = True
            
            # Publish active goals
            self.publish_goals()

    def done(self) -> bool:
        """Return True if mission complete (SUCCESS)"""
        return self.mission_complete

    def plan(self, planner: PotentialFieldsPlanner, max_goals: int = 2) -> None:
        if not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return

        start = (self.pose[0], self.pose[1])
        obstacles = list(self.entities.get_obstacles())
        gates = self.entities.get_gates()
        goals = self.goal_queue[:max_goals]

        out = planner.plan_multi_goal_path(start, goals, obstacles, gates=gates, map_bounds=self.map_bounds)
        self.current_path = list(map(tuple, out["path"]))
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]

    def _fallback_goal_A(self) -> Vec2:
        # Part A fallback: perpendicular to last gate line, forward 50 ft
        x, y, hdg = self.pose
        if self.last_gate_normal is None:
            d = np.array([np.cos(hdg), np.sin(hdg)], dtype=float)
            d = d / (np.linalg.norm(d) + 1e-9)
        else:
            d = self.last_gate_normal

        step = 50.0
        p = np.array([x, y], dtype=float) + d * step
        p[0] = float(np.clip(p[0], 0, self.map_bounds[0]))
        p[1] = float(np.clip(p[1], 0, self.map_bounds[1]))
        return (float(p[0]), float(p[1]))

    def _distance(self, a: Vec2, b: Vec2) -> float:
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def _gate_centers_sorted(self) -> List[Vec2]:
        gates = self.entities.get_gates()
        if not gates:
            return []

        # update last_gate_normal from any gate
        red, green = gates[-1]
        gv = np.array([green[0] - red[0], green[1] - red[1]], dtype=float)
        if np.linalg.norm(gv) > 1e-6:
            n = np.array([-gv[1], gv[0]], dtype=float)
            self.last_gate_normal = n / (np.linalg.norm(n) + 1e-9)

        centers = [((r[0] + g[0]) / 2.0, (r[1] + g[1]) / 2.0) for (r, g) in gates]
        centers.sort(key=lambda p: p[1])  # northbound by y
        return centers

    def _write_goals(self, goals: List[Vec2]):
        self.entities.clear_goals()
        for i, g in enumerate(goals, start=1):
            self.entities.add("goal", g, name=f"goal_wp{i}")
        self.goal_queue = list(goals)

    def _make_circle_waypoints(self, center: Vec2) -> List[Vec2]:
        cx, cy = center
        obstacles = list(self.entities.get_obstacles())

        pts: List[Vec2] = []
        for i in range(self.circle_points):
            th = 2.0 * np.pi * (i / self.circle_points)
            x = cx + self.circle_radius * float(np.cos(th))
            y = cy + self.circle_radius * float(np.sin(th))
            x = float(np.clip(x, 0, self.map_bounds[0]))
            y = float(np.clip(y, 0, self.map_bounds[1]))
            p = (x, y)

            # simple non-collision filter: keep points >2ft from obstacles
            ok = True
            for ox, oy in obstacles:
                if float(np.hypot(p[0] - ox, p[1] - oy)) < 2.0:
                    ok = False
                    break
            if ok:
                pts.append(p)

        # if filtering killed too many points, just return raw 4
        if len(pts) < 2:
            pts = []
            for i in range(self.circle_points):
                th = 2.0 * np.pi * (i / self.circle_points)
                x = cx + self.circle_radius * float(np.cos(th))
                y = cy + self.circle_radius * float(np.sin(th))
                x = float(np.clip(x, 0, self.map_bounds[0]))
                y = float(np.clip(y, 0, self.map_bounds[1]))
        return pts

    def _nearest_survivor(self) -> Optional[Vec2]:
        greens = [e.position for e in self.entities.entities if getattr(e, "type", None) == "green_indicator"]
        if not greens:
            return None
        boat = (self.pose[0], self.pose[1])
        greens.sort(key=lambda p: self._distance(boat, p))
        return tuple(greens[0])


def run_task2(
    entities,
    map_bounds: Vec2,
    *,
    one_shot: bool = True,
    get_pose: Optional[Callable[[], Pose]] = None,
    get_detections: Optional[Callable[[], List[DetectedEntity]]] = None,
    plan: bool = True,
    sleep_dt: float = 0.1,
    max_iters: int = 5000,
) -> Dict:
    """Run Task 2 with the new persistent goal plan architecture"""
    start = entities.get_start()
    mgr = Task2Manager(entities, map_bounds, (float(start[0]), float(start[1]), 0.0))
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
            mgr.plan(planner, max_goals=2)

    # one-shot
    if one_shot:
        step_once()
        return {
            "status": "SUCCESS" if mgr.done() else "RUNNING",
            "phase": mgr.phase.value,
            "goals": list(mgr.goal_queue),
            "report": mgr.report.to_dict(),
            "path": mgr.current_path,
            "velocities": mgr.current_velocities,
            "speeds": mgr.current_speeds,
            "entities": mgr.entities,
        }

    # loop
    if get_pose is None:
        return {"status": "FAILURE", "reason": "loop mode requires get_pose"}

    for _ in range(max_iters):
        step_once()
        if mgr.done():
            return {
                "status": "SUCCESS",
                "phase": mgr.phase.value,
                "goals": list(mgr.goal_queue),
                "report": mgr.report.to_dict(),
                "path": mgr.current_path,
                "velocities": mgr.current_velocities,
                "speeds": mgr.current_speeds,
                "entities": mgr.entities,
            }
        time.sleep(sleep_dt)

    return {"status": "FAILURE", "reason": "max_iters reached"}
