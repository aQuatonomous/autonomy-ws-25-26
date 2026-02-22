#!/usr/bin/env python3
"""
Left-right buoy midpoint steering helper.

This integrates the core behavior from left-right-buoy-nav into the planning stack:
- steer toward the midpoint between a red/green gate pair
- use heading-error proportional control for yaw rate
- reduce forward speed while turning hard
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import math
import numpy as np

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]
Gate = Tuple[Vec3, Vec3]  # (red, green)
Pose = Vec3


@dataclass
class LeftRightBuoyConfig:
    base_speed_m_s: float = 0.9
    min_speed_m_s: float = 0.2
    heading_kp: float = 1.2
    max_yaw_rate_rad_s: float = 0.8
    min_gate_width_m: float = 0.8


def _norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class LeftRightBuoyNavigator:
    """
    Midpoint steering controller for red-left / green-right gate navigation.
    """

    def __init__(self, cfg: Optional[LeftRightBuoyConfig] = None):
        self.cfg = cfg or LeftRightBuoyConfig()

    def _local_left(self, pose: Pose, point: Vec3) -> float:
        dx = float(point[0]) - float(pose[0])
        dy = float(point[1]) - float(pose[1])
        hdg = float(pose[2])
        # ENU world -> body-left component
        return float(-dx * math.sin(hdg) + dy * math.cos(hdg))

    def _score_gate(self, pose: Pose, gate: Gate) -> float:
        red, green = gate
        cx = 0.5 * (float(red[0]) + float(green[0]))
        cy = 0.5 * (float(red[1]) + float(green[1]))
        dx = cx - float(pose[0])
        dy = cy - float(pose[1])
        hdg = float(pose[2])
        # Prefer nearest gate that is ahead of the boat.
        forward = dx * math.cos(hdg) + dy * math.sin(hdg)
        distance = math.hypot(dx, dy)
        if forward < -1.0:
            return float("inf")
        return distance - 0.2 * forward

    def _is_left_right_gate(self, pose: Pose, gate: Gate) -> bool:
        red, green = gate
        gate_width = math.hypot(float(green[0]) - float(red[0]), float(green[1]) - float(red[1]))
        if gate_width < self.cfg.min_gate_width_m:
            return False
        red_left = self._local_left(pose, red)
        green_left = self._local_left(pose, green)
        # left-right-buoy-nav assumption: red on left, green on right
        return red_left > 0.0 and green_left < 0.0

    def _best_gate(self, pose: Pose, gates: List[Gate]) -> Optional[Gate]:
        valid = [g for g in gates if self._is_left_right_gate(pose, g)]
        if not valid:
            return None
        return min(valid, key=lambda g: self._score_gate(pose, g))

    def compute_twist_override(self, pose: Pose, gates: List[Gate]) -> Optional[Dict[str, float]]:
        gate = self._best_gate(pose, gates)
        if gate is None:
            return None

        red, green = gate
        mx = 0.5 * (float(red[0]) + float(green[0]))
        my = 0.5 * (float(red[1]) + float(green[1]))
        desired_hdg = math.atan2(my - float(pose[1]), mx - float(pose[0]))
        err = _norm_angle(desired_hdg - float(pose[2]))

        yaw = float(np.clip(self.cfg.heading_kp * err, -self.cfg.max_yaw_rate_rad_s, self.cfg.max_yaw_rate_rad_s))
        turn_scale = max(0.0, 1.0 - min(abs(err), math.pi) / math.pi)
        speed = self.cfg.min_speed_m_s + (self.cfg.base_speed_m_s - self.cfg.min_speed_m_s) * turn_scale

        return {
            "linear.x": float(speed),
            "linear.y": 0.0,
            "angular.z": yaw,
        }
