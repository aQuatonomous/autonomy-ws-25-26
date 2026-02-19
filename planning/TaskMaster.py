#!/usr/bin/env python3
"""
TaskMaster - Main lifecycle manager for planning tasks

This module creates the correct task manager once, then runs a loop that repeatedly:
1. Updates pose/detections
2. Calls mgr.tick() (updates goal plan) 
3. Publishes goals into EntityList
4. Optionally calls PF
5. Exits only when mgr.done() → SUCCESS

The TaskMaster owns the task lifecycle and ensures each task runs until SUCCESS 
with clear loop behavior without recreating managers each tick.
"""

from __future__ import annotations
from typing import Callable, Dict, List, Optional, Tuple

from Global.types import DetectedEntity
from Global.Task1 import Task1Manager
from Global.Task2 import Task2Manager
from Global.Task3 import Task3Manager
from Local.potential_fields_planner import PotentialFieldsPlanner

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]


class TaskMaster:
    """Main lifecycle manager for planning tasks"""
    
    def __init__(self, *, entities, start: Vec2, task_id: int, map_bounds: Optional[Vec2] = None):
        self.entities = entities
        self.task_id = int(task_id)
        self.planner = PotentialFieldsPlanner(resolution=0.5)

        if hasattr(self.entities, "set_start_position"):
            self.entities.set_start_position(start)
        else:
            self.entities.start_position = start

        start_pose = (float(start[0]), float(start[1]), 0.0)
        if task_id == 1:
            self.manager = Task1Manager(entities, map_bounds, start_pose)
        elif task_id == 2:
            self.manager = Task2Manager(entities, map_bounds, start_pose)
        elif task_id == 3:
            self.manager = Task3Manager(entities, map_bounds, start_pose)
        else:
            raise ValueError(f"Unknown task_id: {task_id}")

    def run_one_shot(
        self,
        get_pose: Optional[Callable[[], Pose]] = None,
        get_detections: Optional[Callable[[], List[DetectedEntity]]] = None,
        use_planning: bool = True,
        map_bounds: Optional[Tuple[float, float]] = None,
    ) -> Dict:
        """
        Run one planning tick: update pose/detections, tick, plan.
        For live ROS use, pass get_pose and get_detections so state is updated each tick.
        map_bounds: (max_x, max_y) from no-go zones; when provided, manager.map_bounds is updated.
        """
        if map_bounds is not None and hasattr(self.manager, "map_bounds"):
            self.manager.map_bounds = (float(map_bounds[0]), float(map_bounds[1]))
        if get_pose is not None:
            pose = get_pose()
            self.manager.update_pose(pose[0], pose[1], pose[2])
        if get_detections is not None:
            for det in get_detections():
                self.manager.add_detected(det)

        self.manager.tick()

        if use_planning:
            try:
                self.manager.plan(self.planner)
            except Exception as e:
                print(f"Planning warning: {e}")

        result = {
            "status": "SUCCESS" if self.manager.done() else "RUNNING",
            "entities": self.manager.entities,
            "path": self.manager.current_path,
            "velocities": self.manager.current_velocities,
            "speeds": self.manager.current_speeds,
        }
        if hasattr(self.manager, "phase"):
            result["phase"] = self.manager.phase.value
        if hasattr(self.manager, "report") and hasattr(self.manager.report, "to_dict"):
            result["report"] = self.manager.report.to_dict()
        return result
