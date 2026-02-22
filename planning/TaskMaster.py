#!/usr/bin/env python3
"""
TaskMaster - Main lifecycle manager for planning tasks.

Default behavior:
- Run one primary task manager selected by task_id.

Integrated Task 4 interrupt behavior (for task_id != 4):
- Task 4 manager runs as a service-boat interrupt overlay.
- Primary task continues until a yield point.
- Then control switches to Task 4 until interrupt work is complete.
- Control returns to the primary task state machine afterward.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple
import math

from Global.types import DetectedEntity
from Global.Task1 import Task1Manager
from Global.Task2 import Task2Manager
from Global.Task3 import Task3Manager
from Global.Task4 import Task4Manager
from Global.Task6 import Task6Manager
from Local.potential_fields_planner import PotentialFieldsPlanner

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]

INTERRUPT_TASK_ID = 4
PRIMARY_GOAL_YIELD_TOLERANCE_M = 2.0
MAX_INTERRUPT_DEFER_TICKS = 15  # ~1.5 s at 10 Hz before forced handover to Task 4


class TaskMaster:
    """Main lifecycle manager for planning tasks with optional Task 4 interrupt overlay."""

    def __init__(self, *, entities, start: Vec2, task_id: int, map_bounds: Optional[Vec2] = None):
        self.entities = entities
        self.task_id = int(task_id)  # Primary task id requested by launch config.
        self.primary_task_id = int(task_id)
        self.active_task_id = int(task_id)  # Task id currently driving cmd_vel.

        self.planner = PotentialFieldsPlanner(resolution=0.5)
        self._interrupt_active = False
        self._goal_yield_tolerance_m = PRIMARY_GOAL_YIELD_TOLERANCE_M
        self._interrupt_defer_ticks = 0

        if hasattr(self.entities, "set_start_position"):
            self.entities.set_start_position(start)
        else:
            self.entities.start_position = start

        h = float(start[2]) if len(start) >= 3 else 0.0
        start_pose = (float(start[0]), float(start[1]), h)

        self.primary_manager = self._build_manager(
            task_id=self.primary_task_id,
            entities=entities,
            map_bounds=map_bounds,
            start_pose=start_pose,
        )

        # Task 4 standalone keeps existing behavior (no overlay).
        if self.primary_task_id == INTERRUPT_TASK_ID:
            self.interrupt_manager = None
        else:
            self.interrupt_manager = Task4Manager(
                entities,
                map_bounds=None,
                start_pose=start_pose,
            )

        # Backward-compatible external access; points to active manager each tick.
        self.manager = self.primary_manager

    def _build_manager(self, *, task_id: int, entities, map_bounds: Optional[Vec2], start_pose: Pose):
        if task_id == 1:
            return Task1Manager(entities, map_bounds, start_pose)
        if task_id == 2:
            return Task2Manager(entities, map_bounds, start_pose)
        if task_id == 3:
            return Task3Manager(entities, map_bounds, start_pose)
        if task_id == 4:
            return Task4Manager(entities, map_bounds=None, start_pose=start_pose)
        if task_id == 6:
            return Task6Manager(entities, map_bounds=map_bounds, start_pose=start_pose)
        raise ValueError(f"Unknown task_id: {task_id}")

    def on_sound_signal(self, signal: int) -> None:
        """
        Forward sound signals to managers that support them.
        This keeps Task 6 sound handling working even while Task 4 interrupt is active.
        """
        if hasattr(self.primary_manager, "on_sound_signal"):
            self.primary_manager.on_sound_signal(signal)
        if self.interrupt_manager is not None and hasattr(self.interrupt_manager, "on_sound_signal"):
            self.interrupt_manager.on_sound_signal(signal)

    def _update_map_bounds(self, manager, map_bounds: Optional[Tuple[float, float]]) -> None:
        # Task 4 intentionally runs without map clipping.
        if map_bounds is None or manager is None or isinstance(manager, Task4Manager):
            return
        if hasattr(manager, "map_bounds"):
            manager.map_bounds = (float(map_bounds[0]), float(map_bounds[1]))

    def _feed_detections(
        self,
        manager,
        detections: List[DetectedEntity],
        *,
        only_yellow_supply: bool = False,
    ) -> None:
        if manager is None:
            return
        for det in detections:
            if only_yellow_supply and getattr(det, "entity_type", "") != "yellow_supply_drop":
                continue
            manager.add_detected(det)

    def _interrupt_has_work(self) -> bool:
        if self.interrupt_manager is None:
            return False
        mgr = self.interrupt_manager

        if bool(getattr(mgr, "pump_on", False)):
            return True
        if len(getattr(mgr, "pending_targets", [])) > 0:
            return True
        if getattr(mgr, "current_target_pos", None) is not None:
            return True
        if getattr(mgr, "pre_interrupt_pose", None) is not None:
            return True
        phase = getattr(mgr, "phase", None)
        if phase is not None and getattr(phase, "value", str(phase)) != "SEARCH":
            return True
        return False

    def _primary_can_yield(self, pose: Pose) -> bool:
        """
        Generic yield condition across task managers:
        - Primary task already done, or
        - No active goals, or
        - Boat is within tolerance of primary next goal.
        """
        if self.primary_manager.done():
            return True

        goals = getattr(self.primary_manager, "goal_queue", None)
        if goals is None:
            goals = getattr(self.entities, "get_goals", lambda: [])()
        if not goals:
            return True

        gx = float(goals[0][0])
        gy = float(goals[0][1])
        dist = math.hypot(float(pose[0]) - gx, float(pose[1]) - gy)
        return dist <= self._goal_yield_tolerance_m

    def _run_manager_tick_and_plan(self, manager, use_planning: bool) -> None:
        manager.tick()
        if use_planning:
            try:
                manager.plan(self.planner)
            except Exception as e:
                print(f"Planning warning: {e}")

    def run_one_shot(
        self,
        get_pose: Optional[Callable[[], Pose]] = None,
        get_detections: Optional[Callable[[], List[DetectedEntity]]] = None,
        use_planning: bool = True,
        map_bounds: Optional[Tuple[float, float]] = None,
    ) -> Dict:
        """
        Run one planning tick:
        1) Update pose/detections for primary (+ Task4 overlay if enabled)
        2) Run active manager (primary or interrupt)
        3) Evaluate interrupt switching for next tick
        """
        self._update_map_bounds(self.primary_manager, map_bounds)
        self._update_map_bounds(self.interrupt_manager, map_bounds)

        pose: Optional[Pose] = None
        if get_pose is not None:
            pose = get_pose()
            self.primary_manager.update_pose(pose[0], pose[1], pose[2])
            if self.interrupt_manager is not None:
                self.interrupt_manager.update_pose(pose[0], pose[1], pose[2])

        detections: List[DetectedEntity] = []
        if get_detections is not None:
            detections = list(get_detections())
            self._feed_detections(self.primary_manager, detections, only_yellow_supply=False)
            self._feed_detections(self.interrupt_manager, detections, only_yellow_supply=True)

        if self.interrupt_manager is not None and self._interrupt_active:
            current_manager = self.interrupt_manager
            self.active_task_id = INTERRUPT_TASK_ID
        else:
            current_manager = self.primary_manager
            self.active_task_id = self.primary_task_id

        self.manager = current_manager
        self._run_manager_tick_and_plan(current_manager, use_planning)

        # Switching decision for next tick.
        if self.interrupt_manager is not None:
            has_interrupt_work = self._interrupt_has_work()
            if self._interrupt_active:
                if not has_interrupt_work:
                    self._interrupt_active = False
                    self._interrupt_defer_ticks = 0
            else:
                if has_interrupt_work:
                    self._interrupt_defer_ticks += 1
                    pose_for_yield = pose
                    if pose_for_yield is None:
                        pose_for_yield = getattr(self.primary_manager, "pose", (0.0, 0.0, 0.0))
                    should_force = self._interrupt_defer_ticks >= MAX_INTERRUPT_DEFER_TICKS
                    if self._primary_can_yield(pose_for_yield) or should_force:
                        self._interrupt_active = True
                        self._interrupt_defer_ticks = 0
                else:
                    self._interrupt_defer_ticks = 0

        if self.interrupt_manager is None:
            done_flag = bool(current_manager.done())
        else:
            done_flag = bool(self.primary_manager.done()) and (not self._interrupt_active) and (not self._interrupt_has_work())

        result = {
            "status": "SUCCESS" if done_flag else "RUNNING",
            "entities": current_manager.entities,
            "path": current_manager.current_path,
            "velocities": current_manager.current_velocities,
            "speeds": current_manager.current_speeds,
            "active_task_id": int(self.active_task_id),
            "primary_task_id": int(self.primary_task_id),
        }
        if hasattr(current_manager, "phase"):
            result["phase"] = current_manager.phase.value
        if hasattr(current_manager, "report") and hasattr(current_manager.report, "to_dict"):
            result["report"] = current_manager.report.to_dict()
        return result
