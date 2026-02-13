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
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple, Dict
import time

from Global.entities import load_entities_from_json
from Global.Task1 import Task1Manager, DetectedEntity
from Global.Task2 import Task2Manager
from Local.potential_fields_planner import PotentialFieldsPlanner

Vec2 = Tuple[float, float]
Pose = Tuple[float, float, float]


@dataclass
class DetectedEntity:
    entity_id: int
    entity_type: str
    position: Vec2


class TaskMaster:
    """Main lifecycle manager for planning tasks"""
    
    def __init__(self, *, entities, map_bounds: Vec2, start: Vec2, task_id: int):
        self.entities = entities
        self.map_bounds = map_bounds
        self.task_id = int(task_id)
        self.planner = PotentialFieldsPlanner(resolution=0.5)

        # Set start position
        if hasattr(self.entities, "set_start_position"):
            self.entities.set_start_position(start)
        else:
            self.entities.start_position = start

        # Create the appropriate manager once
        start_pose = (float(start[0]), float(start[1]), 0.0)
        
        if task_id == 1:
            self.manager = Task1Manager(entities, map_bounds, start_pose)
        elif task_id == 2:
            self.manager = Task2Manager(entities, map_bounds, start_pose)
        else:
            raise ValueError(f"Unknown task_id: {task_id}")

    def run_loop(self, 
                get_pose: Callable[[], Pose],
                get_detections: Callable[[], List[DetectedEntity]], 
                max_iterations: int = 1000,
                sleep_dt: float = 0.1,
                use_planning: bool = True) -> Dict:
        """
        Run the main task loop until completion or failure.
        
        Args:
            get_pose: Function that returns current (x, y, heading)
            get_detections: Function that returns list of DetectedEntity objects
            max_iterations: Maximum number of loop iterations
            sleep_dt: Sleep time between iterations
            use_planning: Whether to call potential fields planning
            
        Returns:
            Dict with status and final results
        """
        
        for iteration in range(max_iterations):
            # 1. Update pose/detections
            pose = get_pose()
            self.manager.update_pose(pose[0], pose[1], pose[2])
            
            detections = get_detections()
            for det in detections:
                self.manager.add_detected(det)
            
            # 2. Update goal plan (this also publishes goals to EntityList)
            self.manager.tick()
            
            # 3. Optionally call potential fields planning
            if use_planning:
                try:
                    self.manager.plan(self.planner)
                except Exception as e:
                    print(f"Planning warning: {e}")
            
            # 4. Check for completion
            if self.manager.done():
                return {
                    "status": "SUCCESS",
                    "iterations": iteration + 1,
                    "entities": self.manager.entities,
                    "path": self.manager.current_path,
                    "velocities": self.manager.current_velocities,
                    "speeds": self.manager.current_speeds,
                }
            
            time.sleep(sleep_dt)
        
        return {
            "status": "FAILURE", 
            "reason": "max_iterations reached",
            "iterations": max_iterations,
            "entities": self.manager.entities,
        }

    def run_one_shot(self, use_planning: bool = True) -> Dict:
        """
        Run one iteration for testing/debugging.
        """
        # Run one tick (this also publishes goals to EntityList)
        self.manager.tick()
        
        # Optionally plan
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
        
        # Add task-specific information
        if hasattr(self.manager, 'phase'):  # Task 2
            result["phase"] = self.manager.phase.value
            result["report"] = self.manager.report.to_dict()
        
        return result


# -------------------------
# Convenience functions
# -------------------------
def run_task1_from_json(json_path: str, **kwargs) -> Dict:
    """Convenience function for Task 1"""
    entities, map_bounds = load_entities_from_json(json_path)
    start = entities.get_start()
    master = TaskMaster(entities=entities, map_bounds=map_bounds, start=start, task_id=1)
    return master.run_one_shot(**kwargs)


def run_task2_from_json(json_path: str, **kwargs) -> Dict:
    """Convenience function for Task 2"""
    entities, map_bounds = load_entities_from_json(json_path)
    start = entities.get_start()
    master = TaskMaster(entities=entities, map_bounds=map_bounds, start=start, task_id=2)
    return master.run_one_shot(**kwargs)


# -------------------------
# Optional CLI demo
# -------------------------
if __name__ == "__main__":
    import os
    
    # Test Task 1
    task1_json = os.path.join("Input_Entities", "task1_entities.json")
    if os.path.exists(task1_json):
        print("Testing Task 1...")
        result = run_task1_from_json(task1_json)
        print(f"Task 1 result: {result['status']}")
        print(f"Goals: {len(result['entities'].get_goals())}")
    
    # Test Task 2  
    task2_json = os.path.join("Input_Entities", "task2_entities.json")
    if os.path.exists(task2_json):
        print("\nTesting Task 2...")
        result = run_task2_from_json(task2_json)
        print(f"Task 2 result: {result['status']}")
        print(f"Goals: {len(result['entities'].get_goals())}")
