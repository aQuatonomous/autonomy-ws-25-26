#!/usr/bin/env python3
"""
Task 4 - service boat interrupt watchdog.

Behavior:
- Continuously watch for yellow service boats (entity type: yellow_supply_drop).
- Queue new service boats by location (dedupe by location, not only id).
- If a service cycle is active, finish the current target first.
- Save pre-interrupt pose (x, y, heading) when the first queued target starts.
- For each queued target: approach -> align -> pump water -> retreat.
- If more service boats are detected during the interrupt, service them before returning.
- After queue is empty, return to pre-interrupt pose and heading, then resume search/watchdog.
"""

from __future__ import annotations

from enum import Enum
from typing import Dict, List, Optional, Set, Tuple
import math
import time
import numpy as np

from Local.potential_fields_planner import PotentialFieldsPlanner
from Global.types import DetectedEntity, Vec3

Vec2 = Tuple[float, float]
Pose = Vec3  # (x, y, heading_rad)

# Task tuning
SERVICE_RANGE_M = 5.0
SERVICE_STANDOFF_M = 4.5
ALIGNMENT_TOLERANCE_RAD = 0.26  # ~15 deg
SERVICE_DURATION_SEC = 3.0
RETREAT_DISTANCE_M = 3.0
SEARCH_GOAL_DISTANCE_M = 100.0
RETURN_POSITION_TOLERANCE_M = 1.5
RETURN_HEADING_TOLERANCE_RAD = 0.12
HEADING_ALIGN_KP = 0.9
MAX_ALIGN_ANGULAR_RATE_RAD_S = 0.6

# Location matching thresholds
SERVICED_LOCATION_MATCH_TOLERANCE_M = 2.0
QUEUE_LOCATION_MATCH_TOLERANCE_M = 2.0


class Phase(Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    SERVICE = "SERVICE"
    RETREAT = "RETREAT"
    RETURN_TO_PRE_INTERRUPT = "RETURN_TO_PRE_INTERRUPT"
    REALIGN_PRE_INTERRUPT_HEADING = "REALIGN_PRE_INTERRUPT_HEADING"


class Task4Manager:
    """Task 4 manager implementing interrupt queue + return-to-pre-interrupt flow."""

    def __init__(self, entities, map_bounds: Optional[Vec2], start_pose: Pose):
        self.entities = entities
        self.map_bounds = None  # Task 4: run in open water (no map clipping)

        self.pose: Pose = (float(start_pose[0]), float(start_pose[1]), float(start_pose[2]))
        self.prev_pos: Optional[Vec2] = None

        self.phase = Phase.SEARCH

        # Interrupt context
        self.pre_interrupt_pose: Optional[Pose] = None
        self.pending_targets: List[Tuple[Optional[int], Vec2]] = []
        self.current_target_id: Optional[int] = None
        self.current_target_pos: Optional[Vec2] = None

        # Completion tracking
        self.serviced_ids: Set[int] = set()
        self.serviced_locations: List[Vec2] = []
        self.serviced_for_report: List[Tuple[int, Vec2]] = []  # (id, pos) for /gs_message_send

        # Service actuation state
        self.service_start_time: Optional[float] = None
        self.retreat_start_pos: Optional[Vec2] = None
        self.pump_on = False

        # Optional direct twist override for heading alignment.
        # Read by global_planner_node if present.
        self.current_twist_override: Optional[Dict[str, float]] = None

        self.goal_queue: List[Vec3] = []
        self.current_path: List[Vec3] = []
        self.current_velocities = np.zeros((0, 2), dtype=float)
        self.current_speeds = np.zeros((0,), dtype=float)

    def update_pose(self, x: float, y: float, heading: float) -> None:
        self.prev_pos = (self.pose[0], self.pose[1])
        self.pose = (float(x), float(y), float(heading))

    def add_detected(self, det: DetectedEntity) -> None:
        # Keep tracked entities fresh by id.
        self.entities.add_or_update(
            det.entity_type,
            det.position,
            det.entity_id,
            name=f"{det.entity_type}_{det.entity_id}",
        )

        # Queue only yellow service boats.
        if det.entity_type != "yellow_supply_drop":
            return
        self._enqueue_target(det.entity_id, (float(det.position[0]), float(det.position[1])))

    def _distance(self, a: Vec2, b: Vec2) -> float:
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def _normalize_angle(self, a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _is_same_location(self, a: Vec2, b: Vec2, tol_m: float) -> bool:
        return self._distance(a, b) <= tol_m

    def _location_is_serviced(self, pos: Vec2) -> bool:
        return any(
            self._is_same_location(pos, s, SERVICED_LOCATION_MATCH_TOLERANCE_M)
            for s in self.serviced_locations
        )

    def _enqueue_target(self, entity_id: Optional[int], pos: Vec2) -> None:
        # Never repeat servicing at a previously serviced location.
        if self._location_is_serviced(pos):
            return

        # If current target matches by id/location, just refresh its position.
        if self.current_target_pos is not None:
            same_active_id = (
                entity_id is not None
                and self.current_target_id is not None
                and entity_id == self.current_target_id
            )
            same_active_loc = self._is_same_location(
                pos, self.current_target_pos, QUEUE_LOCATION_MATCH_TOLERANCE_M
            )
            if same_active_id or same_active_loc:
                self.current_target_pos = pos
                if self.current_target_id is None and entity_id is not None:
                    self.current_target_id = entity_id
                return

        # Update queued entry if matching id.
        if entity_id is not None:
            for i, (qid, qpos) in enumerate(self.pending_targets):
                if qid is not None and qid == entity_id:
                    self.pending_targets[i] = (entity_id, pos)
                    return

        # Update queued entry if same location.
        for i, (qid, qpos) in enumerate(self.pending_targets):
            if self._is_same_location(pos, qpos, QUEUE_LOCATION_MATCH_TOLERANCE_M):
                self.pending_targets[i] = (entity_id if entity_id is not None else qid, pos)
                return

        self.pending_targets.append((entity_id, pos))

    def _refresh_pending_from_entity_list(self) -> None:
        for e in self.entities.get_by_type("yellow_supply_drop"):
            if e.entity_id is None:
                continue
            self._enqueue_target(e.entity_id, (float(e.position[0]), float(e.position[1])))

    def _refresh_current_target_position(self) -> None:
        if self.current_target_pos is None:
            return

        if self.current_target_id is not None:
            for e in self.entities.get_by_type("yellow_supply_drop"):
                if e.entity_id == self.current_target_id:
                    self.current_target_pos = (float(e.position[0]), float(e.position[1]))
                    return

        # Fallback: keep target aligned with nearest pending location if very close.
        nearest_idx = None
        nearest_dist = float("inf")
        for i, (_, p) in enumerate(self.pending_targets):
            d = self._distance(self.current_target_pos, p)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i
        if nearest_idx is not None and nearest_dist <= QUEUE_LOCATION_MATCH_TOLERANCE_M:
            qid, qpos = self.pending_targets.pop(nearest_idx)
            self.current_target_id = qid if qid is not None else self.current_target_id
            self.current_target_pos = qpos

    def _set_single_goal(self, goal: Vec3) -> None:
        self.entities.clear_goals()
        self.entities.add("goal", goal, name="goal_wp1")
        self.goal_queue = [goal]

    def _clear_goals(self) -> None:
        self.entities.clear_goals()
        self.goal_queue = []

    def _search_goal(self) -> Vec3:
        x, y, hdg = self.pose
        dx = SEARCH_GOAL_DISTANCE_M * math.cos(hdg)
        dy = SEARCH_GOAL_DISTANCE_M * math.sin(hdg)
        return (x + dx, y + dy, hdg)

    def _approach_goal(self, target_pos: Vec2) -> Vec3:
        x, y, _ = self.pose
        tx, ty = target_pos
        angle_to_target = math.atan2(ty - y, tx - x)
        gx = tx - SERVICE_STANDOFF_M * math.cos(angle_to_target)
        gy = ty - SERVICE_STANDOFF_M * math.sin(angle_to_target)
        return (gx, gy, angle_to_target)

    def _retreat_goal(self) -> Vec3:
        x, y, hdg = self.pose
        dx = RETREAT_DISTANCE_M * math.cos(hdg)
        dy = RETREAT_DISTANCE_M * math.sin(hdg)
        return (x - dx, y - dy, hdg)

    def _activate_next_target(self) -> bool:
        if not self.pending_targets:
            return False
        x, y, _ = self.pose
        pos = (x, y)
        best_idx = min(
            range(len(self.pending_targets)),
            key=lambda i: self._distance(pos, self.pending_targets[i][1]),
        )
        target_id, target_pos = self.pending_targets.pop(best_idx)
        self.current_target_id = target_id
        self.current_target_pos = target_pos
        self.phase = Phase.APPROACH
        return True

    def _record_serviced_current_target(self) -> None:
        if self.current_target_pos is None:
            return

        if self._location_is_serviced(self.current_target_pos):
            return

        self.serviced_locations.append(self.current_target_pos)
        if self.current_target_id is not None:
            self.serviced_ids.add(self.current_target_id)
            report_id = int(self.current_target_id)
        else:
            report_id = len(self.serviced_locations)
        self.serviced_for_report.append((report_id, self.current_target_pos))

    def _finish_current_target_and_advance(self) -> None:
        self.current_target_id = None
        self.current_target_pos = None
        if self.pending_targets:
            self._activate_next_target()
        elif self.pre_interrupt_pose is not None:
            self.phase = Phase.RETURN_TO_PRE_INTERRUPT
        else:
            self.phase = Phase.SEARCH

    def _obstacles_for_planner(self) -> List[Vec3]:
        obstacle_types = {
            "red_buoy", "green_buoy", "green_pole_buoy", "red_pole_buoy",
            "black_buoy", "yellow_buoy",
            "red_indicator", "green_indicator",
            "yellow_supply_drop", "black_supply_drop",
        }
        out: List[Vec3] = []

        for e in self.entities.entities:
            if e.type not in obstacle_types:
                continue

            if e.type == "yellow_supply_drop" and self.current_target_pos is not None:
                same_id = (
                    self.current_target_id is not None
                    and e.entity_id is not None
                    and e.entity_id == self.current_target_id
                )
                same_loc = self._is_same_location(
                    (float(e.position[0]), float(e.position[1])),
                    self.current_target_pos,
                    QUEUE_LOCATION_MATCH_TOLERANCE_M,
                )
                if same_id or same_loc:
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
        pos = (x, y)

        self._refresh_pending_from_entity_list()

        if self.phase == Phase.SEARCH:
            if self.pending_targets:
                # First entry into interrupt flow: remember where we were.
                if self.pre_interrupt_pose is None:
                    self.pre_interrupt_pose = (x, y, hdg)
                self._activate_next_target()
            else:
                self._set_single_goal(self._search_goal())

        elif self.phase == Phase.APPROACH:
            self._refresh_current_target_position()
            if self.current_target_pos is None:
                self._finish_current_target_and_advance()
                return

            dist = self._distance(pos, self.current_target_pos)
            if dist <= SERVICE_RANGE_M:
                self.phase = Phase.ALIGN
                self._clear_goals()
            else:
                self._set_single_goal(self._approach_goal(self.current_target_pos))

        elif self.phase == Phase.ALIGN:
            self._refresh_current_target_position()
            if self.current_target_pos is None:
                self._finish_current_target_and_advance()
                return

            tx, ty = self.current_target_pos
            dist = self._distance(pos, self.current_target_pos)
            angle_to_target = math.atan2(ty - y, tx - x)
            heading_error = self._normalize_angle(angle_to_target - hdg)

            if dist > SERVICE_RANGE_M + 0.5:
                self.phase = Phase.APPROACH
                self.current_twist_override = None
                self._set_single_goal(self._approach_goal(self.current_target_pos))
                return

            if abs(heading_error) <= ALIGNMENT_TOLERANCE_RAD and dist <= SERVICE_RANGE_M:
                self.phase = Phase.SERVICE
                self.service_start_time = time.time()
                self.pump_on = True
                self._clear_goals()
            else:
                # Hold position and rotate bow toward target.
                self._set_single_goal((x, y, angle_to_target))
                omega = float(np.clip(
                    HEADING_ALIGN_KP * heading_error,
                    -MAX_ALIGN_ANGULAR_RATE_RAD_S,
                    MAX_ALIGN_ANGULAR_RATE_RAD_S,
                ))
                self.current_twist_override = {
                    "linear.x": 0.0,
                    "linear.y": 0.0,
                    "angular.z": omega,
                }

        elif self.phase == Phase.SERVICE:
            # Keep boat settled while pumping.
            self._set_single_goal((x, y, hdg))
            elapsed = time.time() - (self.service_start_time or time.time())
            if elapsed >= SERVICE_DURATION_SEC:
                self.pump_on = False
                self._record_serviced_current_target()
                self.retreat_start_pos = pos
                self.phase = Phase.RETREAT

        elif self.phase == Phase.RETREAT:
            if self.retreat_start_pos is None:
                self.retreat_start_pos = pos

            dist_moved = self._distance(self.retreat_start_pos, pos)
            if dist_moved >= RETREAT_DISTANCE_M:
                self.retreat_start_pos = None
                self._finish_current_target_and_advance()
            else:
                self._set_single_goal(self._retreat_goal())

        elif self.phase == Phase.RETURN_TO_PRE_INTERRUPT:
            # If new boats appeared while returning, service them before completing return.
            if self.pending_targets:
                self._activate_next_target()
                return

            if self.pre_interrupt_pose is None:
                self.phase = Phase.SEARCH
                return

            px, py, ph = self.pre_interrupt_pose
            if self._distance(pos, (px, py)) <= RETURN_POSITION_TOLERANCE_M:
                self.phase = Phase.REALIGN_PRE_INTERRUPT_HEADING
                self._clear_goals()
            else:
                self._set_single_goal((float(px), float(py), float(ph)))

        elif self.phase == Phase.REALIGN_PRE_INTERRUPT_HEADING:
            if self.pending_targets:
                self._activate_next_target()
                return

            if self.pre_interrupt_pose is None:
                self.phase = Phase.SEARCH
                return

            target_heading = float(self.pre_interrupt_pose[2])
            err = self._normalize_angle(target_heading - hdg)
            if abs(err) <= RETURN_HEADING_TOLERANCE_RAD:
                self.pre_interrupt_pose = None
                self.phase = Phase.SEARCH
                self.current_twist_override = None
                self._set_single_goal(self._search_goal())
            else:
                self._clear_goals()
                omega = float(np.clip(
                    HEADING_ALIGN_KP * err,
                    -MAX_ALIGN_ANGULAR_RATE_RAD_S,
                    MAX_ALIGN_ANGULAR_RATE_RAD_S,
                ))
                self.current_twist_override = {
                    "linear.x": 0.0,
                    "linear.y": 0.0,
                    "angular.z": omega,
                }

        self.prev_pos = (x, y)

    def done(self) -> bool:
        # Task 4 watchdog remains active for the full run.
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
            map_bounds=None,
        )
        self.current_path = [(float(p[0]), float(p[1]), 0.0) for p in out["path"]]
        self.current_velocities = out["velocities"]
        self.current_speeds = out["speeds"]
