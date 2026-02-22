#!/usr/bin/env python3
"""
Task 6 - Harbor Alert (sound signal interrupt).

Behavior:
- Continuously track yellow buoys from perception.
- On sound signal:
  - 1 blast -> divert to east-most yellow buoy.
  - 2 blasts -> divert to west-most yellow buoy.
- Save current pose before divert.
- Navigate near (not on top of) target buoy using potential fields.
- Hold near target for 5 seconds.
- Return to saved pose and realign heading.
- Resume normal patrol behavior.
"""

from __future__ import annotations

from enum import Enum
from typing import Dict, List, Optional, Tuple
import math
import time
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner
from Global.types import DetectedEntity, Vec2, Vec3

Pose = Vec3  # (x, y, heading_rad)

# Task tuning
PATROL_FORWARD_DIST_M = 30.0
TARGET_STANDOFF_M = 3.0
TARGET_NEAR_DIST_M = 4.0
TARGET_HOLD_SEC = 5.0
RETURN_POSITION_TOLERANCE_M = 1.5
RETURN_HEADING_TOLERANCE_RAD = 0.12
HEADING_ALIGN_KP = 0.9
MAX_ALIGN_ANGULAR_RATE_RAD_S = 0.6


class Phase(Enum):
    PATROL = "PATROL"
    DIVERT_TO_TARGET = "DIVERT_TO_TARGET"
    HOLD_AT_TARGET = "HOLD_AT_TARGET"
    RETURN_TO_PRE_INTERRUPT = "RETURN_TO_PRE_INTERRUPT"
    REALIGN_HEADING = "REALIGN_HEADING"


class Task6Manager:
    """Task 6 manager with interrupt/return state machine."""

    def __init__(self, entities, map_bounds: Optional[Vec2], start_pose: Pose):
        self.entities = entities
        self.map_bounds = (float(map_bounds[0]), float(map_bounds[1])) if map_bounds else None

        self.pose: Pose = (float(start_pose[0]), float(start_pose[1]), float(start_pose[2]))
        self.prev_pos: Optional[Vec2] = None

        self.phase = Phase.PATROL
        self.pending_signal: Optional[int] = None
        self.active_signal: Optional[int] = None

        self.pre_interrupt_pose: Optional[Pose] = None
        self.target_buoy_id: Optional[int] = None
        self.target_buoy_pos: Optional[Vec3] = None
        self.hold_start_time: Optional[float] = None

        # Optional direct twist override for heading alignment.
        # Read by global_planner_node if present.
        self.current_twist_override: Optional[Dict[str, float]] = None

        self.goal_queue: List[Vec3] = []
        self.current_path: List[Vec3] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float) -> None:
        if self.prev_pos is None:
            self.prev_pos = (self.pose[0], self.pose[1])
        else:
            self.prev_pos = (self.pose[0], self.pose[1])
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        """Add/update tracked entities by id."""
        already = any(getattr(e, "entity_id", None) == det.entity_id for e in self.entities.entities)
        if not already:
            self.entities.add_or_update(
                det.entity_type,
                det.position,
                det.entity_id,
                name=f"{det.entity_type}_{det.entity_id}",
            )

    def on_sound_signal(self, signal: int) -> None:
        """Called by node callback on /sound_signal_interupt for Task 6."""
        if signal not in (1, 2):
            return
        # Start immediately if idle; otherwise queue latest signal.
        if self.phase == Phase.PATROL and self._start_interrupt(signal):
            self.pending_signal = None
            return
        self.pending_signal = signal

    def _distance(self, a: Vec2, b: Vec2) -> float:
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def _normalize_angle(self, a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _clip(self, x: float, y: float) -> Tuple[float, float]:
        if self.map_bounds is None:
            return (float(x), float(y))
        return (
            float(np.clip(x, 0.0, self.map_bounds[0])),
            float(np.clip(y, 0.0, self.map_bounds[1])),
        )

    def _set_single_goal(self, goal: Vec3) -> None:
        self.entities.clear_goals()
        self.entities.add("goal", goal, name="goal_wp1")
        self.goal_queue = [goal]

    def _clear_goals(self) -> None:
        self.entities.clear_goals()
        self.goal_queue = []

    def _patrol_goal(self) -> Vec3:
        x, y, hdg = self.pose
        gx = x + PATROL_FORWARD_DIST_M * math.cos(hdg)
        gy = y + PATROL_FORWARD_DIST_M * math.sin(hdg)
        gx, gy = self._clip(gx, gy)
        return (gx, gy, hdg)

    def _pick_target_by_signal(self, signal: int) -> Optional[Tuple[Optional[int], Vec3]]:
        yellows = self.entities.get_by_type("yellow_buoy")
        if not yellows:
            return None
        if signal == 1:
            chosen = max(yellows, key=lambda e: float(e.position[0]))  # East-most
        else:
            chosen = min(yellows, key=lambda e: float(e.position[0]))  # West-most
        return (chosen.entity_id, (float(chosen.position[0]), float(chosen.position[1]), 0.0))

    def _refresh_target_position(self) -> None:
        """Refresh tracked target position using entity id, fallback to east/west pick."""
        if self.active_signal not in (1, 2):
            return
        yellows = self.entities.get_by_type("yellow_buoy")
        if self.target_buoy_id is not None:
            for e in yellows:
                if e.entity_id == self.target_buoy_id:
                    self.target_buoy_pos = (float(e.position[0]), float(e.position[1]), 0.0)
                    return
        picked = self._pick_target_by_signal(self.active_signal)
        if picked is not None:
            self.target_buoy_id, self.target_buoy_pos = picked

    def _start_interrupt(self, signal: int) -> bool:
        picked = self._pick_target_by_signal(signal)
        if picked is None:
            return False
        self.pre_interrupt_pose = (float(self.pose[0]), float(self.pose[1]), float(self.pose[2]))
        self.target_buoy_id, self.target_buoy_pos = picked
        self.active_signal = int(signal)
        self.hold_start_time = None
        self.phase = Phase.DIVERT_TO_TARGET
        return True

    def _finish_interrupt(self) -> None:
        self.pre_interrupt_pose = None
        self.target_buoy_id = None
        self.target_buoy_pos = None
        self.active_signal = None
        self.hold_start_time = None
        self.current_twist_override = None
        self.phase = Phase.PATROL

    def _approach_goal_near_target(self, target: Vec3) -> Vec3:
        x, y, _ = self.pose
        tx, ty = target[0], target[1]
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return (x, y, self.pose[2])
        # Place goal TARGET_STANDOFF_M away from target, on boat-target line.
        gx = tx - TARGET_STANDOFF_M * (dx / dist)
        gy = ty - TARGET_STANDOFF_M * (dy / dist)
        gx, gy = self._clip(gx, gy)
        return (gx, gy, math.atan2(ty - y, tx - x))

    def _obstacles_for_planner(self) -> List[Vec3]:
        obstacle_types = {
            "red_buoy", "green_buoy", "green_pole_buoy", "red_pole_buoy",
            "black_buoy",
            "yellow_buoy",
            "red_indicator", "green_indicator",
            "yellow_supply_drop", "black_supply_drop",
        }
        out: List[Vec3] = []
        for e in self.entities.entities:
            if e.type not in obstacle_types:
                continue
            if e.type == "yellow_buoy":
                # Exclude current target yellow from repulsion so planner can approach standoff.
                if self.target_buoy_id is not None and e.entity_id == self.target_buoy_id:
                    continue
                if self.target_buoy_id is None and self.target_buoy_pos is not None:
                    if self._distance(
                        (float(e.position[0]), float(e.position[1])),
                        (float(self.target_buoy_pos[0]), float(self.target_buoy_pos[1])),
                    ) < 0.5:
                        continue
            out.append((float(e.position[0]), float(e.position[1]), 0.0))

        out.extend(
            (float(p[0]), float(p[1]), 0.0)
            for p in self.entities.get_no_go_obstacle_points(
                map_bounds=self.map_bounds,
                boat_heading_rad=self.pose[2],
            )
        )
        return out

    def tick(self) -> None:
        self.current_twist_override = None
        x, y, hdg = self.pose

        # If a signal was queued while busy, process it when back to PATROL.
        if self.phase == Phase.PATROL and self.pending_signal in (1, 2):
            if self._start_interrupt(self.pending_signal):
                self.pending_signal = None

        if self.phase == Phase.PATROL:
            self._set_single_goal(self._patrol_goal())

        elif self.phase == Phase.DIVERT_TO_TARGET:
            self._refresh_target_position()
            if self.target_buoy_pos is None:
                # No yellow available yet: keep moving while waiting for yellow detections.
                self._set_single_goal(self._patrol_goal())
                return

            d = self._distance((x, y), (self.target_buoy_pos[0], self.target_buoy_pos[1]))
            if d <= TARGET_NEAR_DIST_M:
                self.phase = Phase.HOLD_AT_TARGET
                self.hold_start_time = time.time()
                self._clear_goals()
            else:
                self._set_single_goal(self._approach_goal_near_target(self.target_buoy_pos))

        elif self.phase == Phase.HOLD_AT_TARGET:
            self._clear_goals()
            if self.hold_start_time is None:
                self.hold_start_time = time.time()
            if (time.time() - self.hold_start_time) >= TARGET_HOLD_SEC:
                self.phase = Phase.RETURN_TO_PRE_INTERRUPT

        elif self.phase == Phase.RETURN_TO_PRE_INTERRUPT:
            if self.pre_interrupt_pose is None:
                self._finish_interrupt()
                self._set_single_goal(self._patrol_goal())
                return

            px, py, ph = self.pre_interrupt_pose
            if self._distance((x, y), (px, py)) <= RETURN_POSITION_TOLERANCE_M:
                self.phase = Phase.REALIGN_HEADING
                self._clear_goals()
            else:
                self._set_single_goal((float(px), float(py), float(ph)))

        elif self.phase == Phase.REALIGN_HEADING:
            self._clear_goals()
            if self.pre_interrupt_pose is None:
                self._finish_interrupt()
                self._set_single_goal(self._patrol_goal())
                return

            target_heading = float(self.pre_interrupt_pose[2])
            err = self._normalize_angle(target_heading - hdg)
            if abs(err) <= RETURN_HEADING_TOLERANCE_RAD:
                self._finish_interrupt()
                self._set_single_goal(self._patrol_goal())
            else:
                omega = np.clip(
                    HEADING_ALIGN_KP * err,
                    -MAX_ALIGN_ANGULAR_RATE_RAD_S,
                    MAX_ALIGN_ANGULAR_RATE_RAD_S,
                )
                self.current_twist_override = {
                    "linear.x": 0.0,
                    "linear.y": 0.0,
                    "angular.z": float(omega),
                }

    def done(self) -> bool:
        # Task 6 remains active to keep handling new sound interrupts.
        return False

    def plan(self, planner: PotentialFieldsPlanner) -> None:
        if self.current_twist_override is not None or not self.goal_queue:
            self.current_path = []
            self.current_velocities = np.zeros((0, 2), dtype=float)
            self.current_speeds = np.zeros((0,), dtype=float)
            return

        start = (self.pose[0], self.pose[1])
        obstacles = self._obstacles_for_planner()
        goals_xy = [(g[0], g[1]) for g in self.goal_queue[:1]]
        out = planner.plan_multi_goal_path(
            start,
            goals_xy,
            obstacles,
            gates=None,
            map_bounds=self.map_bounds,
        )
        self.current_path = [(float(p[0]), float(p[1]), 0.0) for p in out["path"]]
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]
