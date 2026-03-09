import json
import os
import math
from typing import Optional, Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import ParameterType
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from global_frame.msg import BoatPose


RED_MIN_DISTANCE = 2.0  # meters minimum distance between boat and red buoy
GREEN_MIN_DISTANCE = 2.0  # meters minimum distance between boat and green buoy
GOAL_DISTANCE = 5.0  # meters distance ahead of midpoint when both buoys seen
STRAIGHT_DISTANCE = 10.0  # meters ahead when no buoys detected (go straight)
DONE_THRESHOLD = 2.0  # meters; if boat within this of goal, gate passed → go straight


def left_right_buoy_goal_local(
    red_distance_m: Optional[float],
    red_bearing_rad: Optional[float],
    green_distance_m: Optional[float],
    green_bearing_rad: Optional[float],
) -> tuple[float, float]:
    """
    Left/right buoy nav in local frame: red on left of boat, green on right.
    Returns goal (goal_local_x, goal_local_y) in boat frame (+x forward, +y left).
    - Both seen: goal at midpoint ahead.
    - Only green: goal to the left of green by at least GREEN_MIN_DISTANCE.
    - Only red: goal to the right of red by at least RED_MIN_DISTANCE.
    - Neither: (0, 0).
    """
    red_seen = red_distance_m is not None and red_bearing_rad is not None
    green_seen = green_distance_m is not None and green_bearing_rad is not None

    if red_seen and green_seen:
        red_x = red_distance_m * math.cos(red_bearing_rad)
        red_y = red_distance_m * math.sin(red_bearing_rad)
        green_x = green_distance_m * math.cos(green_bearing_rad)
        green_y = green_distance_m * math.sin(green_bearing_rad)
        goal_local_x = (red_x + green_x) / 2.0
        goal_local_y = (red_y + green_y) / 2.0 + GOAL_DISTANCE
        return (goal_local_x, goal_local_y)

    if green_seen:
        # Only green: go to the left of it (green on our right) by at least GREEN_MIN_DISTANCE
        # Boat frame +y = left, so goal = green position + (0, GREEN_MIN_DISTANCE)
        green_x = green_distance_m * math.cos(green_bearing_rad)
        green_y = green_distance_m * math.sin(green_bearing_rad)
        return (green_x, green_y + GREEN_MIN_DISTANCE)

    if red_seen:
        # Only red: go to the right of it (red on our left) by at least RED_MIN_DISTANCE
        # Boat frame -y = right, so goal = red position + (0, -RED_MIN_DISTANCE)
        red_x = red_distance_m * math.cos(red_bearing_rad)
        red_y = red_distance_m * math.sin(red_bearing_rad)
        return (red_x, red_y - RED_MIN_DISTANCE)

    return (None, None) 
    


def local_to_global(
    goal_local_x: float,
    goal_local_y: float,
    boat_east: float,
    boat_north: float,
    boat_heading_rad: float,
) -> tuple[float, float]:
    """
    Convert goal from boat frame to map frame (east, north).
    Boat frame: +x forward, +y left. Map: ENU (east, north).
    """
    c = math.cos(boat_heading_rad)
    s = math.sin(boat_heading_rad)
    global_east = boat_east + c * goal_local_x - s * goal_local_y
    global_north = boat_north + s * goal_local_x + c * goal_local_y
    return (global_east, global_north)


class LR_Bouy(Node):
    def __init__(self):
        super().__init__('LR_Bouy')

        # QoS profile for CV detection (sensor data: BEST_EFFORT)
        self._qos_cv = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # QoS profile for boat pose (RELIABLE)
        self._qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Boat pose state
        self._boat_east: Optional[float] = None
        self._boat_north: Optional[float] = None
        self._boat_heading_rad: Optional[float] = None

        # Red and green buoy detections (distance_m, bearing_rad)
        self._red: Tuple[Optional[float], Optional[float]] = (None, None)
        self._green: Tuple[Optional[float], Optional[float]] = (None, None)

        # Subscriptions
        self._cv_sub = self.create_subscription(
            String,
            '/combined/detection_info_with_distance',
            self._detection_callback,
            self._qos_cv,
        )
        self._pose_sub = self.create_subscription(
            BoatPose,
            '/boat_pose',
            self._boat_pose_callback,
            self._qos_reliable,
        )

        # Publisher for MAVROS local position setpoint (ENU meters, for Gazebo)
        self._setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
        )
        # Optional: JSON goal for debugging / integration
        self._goal_pub = self.create_publisher(String, '/lr_buoy_goal', 10)

        # Timer for periodic goal computation
        self._timer = self.create_timer(0.1, self._timer_callback)

        self.get_logger().info('LR_Bouy node initialized: subscribing to /combined/detection_info_with_distance and /boat_pose')

    def _boat_pose_callback(self, msg: BoatPose) -> None:
        """Update boat pose from /boat_pose."""
        self._boat_east = msg.east
        self._boat_north = msg.north
        self._boat_heading_rad = msg.heading_rad

    def _detection_callback(self, msg: String) -> None:
        """Parse CV detections and extract red/green buoys."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'LR Buoy: invalid JSON: {e}')
            return

        detections = data.get('detections', [])
        red_dist, red_bearing_deg = None, None
        green_dist, green_bearing_deg = None, None

        # Extract red and green buoys from detections
        for det in detections:
            name = (det.get('class_name') or '').strip().lower()
            d = det.get('distance_m')
            b = det.get('bearing_deg')

            if d is None or b is None:
                continue

            if name in ('red_buoy', 'red_pole_buoy'):
                red_dist = float(d)
                red_bearing_deg = float(b)
            elif name in ('green_buoy', 'green_pole_buoy'):
                green_dist = float(d)
                green_bearing_deg = float(b)

        # Convert bearing_deg to radians and store
        self._red = (red_dist, math.radians(red_bearing_deg) if red_bearing_deg is not None else None)
        self._green = (green_dist, math.radians(green_bearing_deg) if green_bearing_deg is not None else None)

    def _timer_callback(self) -> None:
        """Compute goal in local frame, convert to global, publish to MAVROS and /lr_buoy_goal."""
        # Require boat pose
        if self._boat_east is None or self._boat_north is None or self._boat_heading_rad is None:
            self.get_logger().warn_throttle(5.0, 'Boat pose not yet received')
            return

        red_dist, red_bearing_rad = self._red
        green_dist, green_bearing_rad = self._green
        goal_local = left_right_buoy_goal_local(red_dist, red_bearing_rad, green_dist, green_bearing_rad)

        # When nothing detected: go straight ahead
        if goal_local == (None, None):
            goal_east = self._boat_east + STRAIGHT_DISTANCE * math.cos(self._boat_heading_rad)
            goal_north = self._boat_north + STRAIGHT_DISTANCE * math.sin(self._boat_heading_rad)
        else:
            goal_local_x, goal_local_y = goal_local
            goal_east, goal_north = local_to_global(
                goal_local_x,
                goal_local_y,
                self._boat_east,
                self._boat_north,
                self._boat_heading_rad,
            )
            # 1. Goal behind us: gate passed → go straight
            if goal_local_x < 0:
                goal_east = self._boat_east + STRAIGHT_DISTANCE * math.cos(self._boat_heading_rad)
                goal_north = self._boat_north + STRAIGHT_DISTANCE * math.sin(self._boat_heading_rad)
            else:
                # 2. Within threshold: gate passed → go straight
                dist_to_goal = math.hypot(
                    goal_east - self._boat_east,
                    goal_north - self._boat_north,
                )
                if dist_to_goal < DONE_THRESHOLD:
                    goal_east = self._boat_east + STRAIGHT_DISTANCE * math.cos(self._boat_heading_rad)
                    goal_north = self._boat_north + STRAIGHT_DISTANCE * math.sin(self._boat_heading_rad)

        # Publish to MAVROS setpoint_position/local (ENU meters for Gazebo)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = goal_east
        pose_msg.pose.position.y = goal_north
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self._setpoint_pub.publish(pose_msg)

        # Optional: JSON goal for debugging
        goal_msg = {
            'goal_east': goal_east,
            'goal_north': goal_north,
            'boat_east': self._boat_east,
            'boat_north': self._boat_north,
            'boat_heading_rad': self._boat_heading_rad,
            'red_detected': red_dist is not None,
            'green_detected': green_dist is not None,
        }
        str_msg = String()
        str_msg.data = json.dumps(goal_msg)
        self._goal_pub.publish(str_msg)

        self.get_logger().debug(f'Goal: ({goal_east:.2f}, {goal_north:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = LR_Bouy()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('LR_Bouy shutting down...')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


