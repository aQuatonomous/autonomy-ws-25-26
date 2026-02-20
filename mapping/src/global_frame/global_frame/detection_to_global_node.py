#!/usr/bin/env python3
"""
Detection-to-global node: boat pose + LiDAR + CV detections -> global (east, north) in map frame.

Subscribes to:
  - /boat_pose (global_frame/BoatPose)
  - /tracked_buoys_json (std_msgs/String)
  - /combined/detection_info_with_distance (std_msgs/String)

Publishes:
  - /global_detections (global_frame/GlobalDetectionArray)

Uses latest boat pose for each detection. Heading convention: 0 = East, positive = CCW.
Boat-relative bearing: 0 = boat forward (+X). CV bearing_deg: 0 = forward, + = right -> we use bearing_rad = -rad(deg) to match base_link.
"""

import json
import math
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from builtin_interfaces.msg import Time

from global_frame.msg import BoatPose, GlobalDetection, GlobalDetectionArray


def _normalize_angle(angle_rad: float) -> float:
    """Normalize angle to [-π, π]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class DetectionToGlobalNode(Node):
    def __init__(self):
        super().__init__("detection_to_global_node")

        self.declare_parameter("boat_pose_topic", "/boat_pose")
        self.declare_parameter("tracked_buoys_topic", "/tracked_buoys_json")
        self.declare_parameter("detection_info_topic", "/combined/detection_info_with_distance")
        self.declare_parameter("global_detections_topic", "/global_detections")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("merge_by_proximity_m", 0.0)  # 0 = disabled; >0 merge detections within this distance

        self._boat_pose_topic = self.get_parameter("boat_pose_topic").value
        self._tracked_buoys_topic = self.get_parameter("tracked_buoys_topic").value
        self._detection_info_topic = self.get_parameter("detection_info_topic").value
        self._global_detections_topic = self.get_parameter("global_detections_topic").value
        self._map_frame = self.get_parameter("map_frame").value
        self._merge_by_proximity_m = float(self.get_parameter("merge_by_proximity_m").value)

        self._boat_east: Optional[float] = None
        self._boat_north: Optional[float] = None
        self._boat_heading_rad: Optional[float] = None
        self._boat_stamp: Optional[Time] = None
        self._last_lidar_detections: list = []
        self._last_cv_detections: list = []

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._boat_pose_sub = self.create_subscription(
            BoatPose,
            self._boat_pose_topic,
            self._boat_pose_callback,
            qos,
        )
        self._lidar_sub = self.create_subscription(
            String,
            self._tracked_buoys_topic,
            self._lidar_callback,
            qos,
        )
        self._cv_sub = self.create_subscription(
            String,
            self._detection_info_topic,
            self._cv_callback,
            qos,
        )
        self._global_pub = self.create_publisher(
            GlobalDetectionArray,
            self._global_detections_topic,
            qos,
        )

        self.get_logger().info(
            f"detection_to_global: pose={self._boat_pose_topic}, lidar={self._tracked_buoys_topic}, cv={self._detection_info_topic} -> {self._global_detections_topic}"
        )

    def _boat_pose_callback(self, msg: BoatPose) -> None:
        self._boat_east = msg.east
        self._boat_north = msg.north
        self._boat_heading_rad = msg.heading_rad
        self._boat_stamp = msg.header.stamp

    def _to_global(self, range_m: float, bearing_rad: float) -> tuple[float, float]:
        """Convert boat-relative (range, bearing) to global (east, north)."""
        if self._boat_east is None or self._boat_north is None or self._boat_heading_rad is None:
            return (0.0, 0.0)
        alpha = _normalize_angle(self._boat_heading_rad + bearing_rad)
        east = self._boat_east + range_m * math.cos(alpha)
        north = self._boat_north + range_m * math.sin(alpha)
        return (east, north)

    def _merge_by_proximity(self, detections: list) -> list:
        """Merge detections within merge_by_proximity_m; return merged list (one per cluster)."""
        if not detections or self._merge_by_proximity_m <= 0.0:
            return detections
        D = self._merge_by_proximity_m
        n = len(detections)
        parent = list(range(n))

        def find(i: int) -> int:
            if parent[i] != i:
                parent[i] = find(parent[i])
            return parent[i]

        def union(i: int, j: int) -> None:
            pi, pj = find(i), find(j)
            if pi != pj:
                parent[pi] = pj

        for i in range(n):
            for j in range(i + 1, n):
                di = detections[i]
                dj = detections[j]
                dist = math.hypot(di.east - dj.east, di.north - dj.north)
                if dist <= D:
                    union(i, j)

        groups: dict[int, list[int]] = {}
        for i in range(n):
            p = find(i)
            groups.setdefault(p, []).append(i)

        merged = []
        for indices in groups.values():
            dets = [detections[i] for i in indices]
            east = sum(d.east for d in dets) / len(dets)
            north = sum(d.north for d in dets) / len(dets)
            range_m = sum(d.range_m for d in dets) / len(dets)
            bearing_global_rad = _normalize_angle(math.atan2(
                north - (self._boat_north or 0),
                east - (self._boat_east or 0),
            ))
            class_name = "unknown"
            class_id = 255
            for d in dets:
                if d.source == "vision" and d.class_name != "unknown":
                    class_name = d.class_name
                    class_id = d.class_id
                    break
            g = GlobalDetection()
            g.header.stamp = dets[0].header.stamp
            g.header.frame_id = self._map_frame
            g.east = east
            g.north = north
            g.range_m = range_m
            g.bearing_global_rad = bearing_global_rad
            g.source = "fused"
            g.class_name = class_name
            g.class_id = class_id
            g.id = dets[0].id
            merged.append(g)
        return merged

    def _publish_combined(self) -> None:
        """Publish merged LiDAR + CV global detections (optionally fuse by proximity)."""
        combined = self._last_lidar_detections + self._last_cv_detections
        if not combined:
            return
        if self._merge_by_proximity_m > 0.0:
            combined = self._merge_by_proximity(combined)
        out = GlobalDetectionArray()
        out.header.stamp = self._boat_stamp or self.get_clock().now().to_msg()
        out.header.frame_id = self._map_frame
        out.detections = combined
        self._global_pub.publish(out)

    def _lidar_callback(self, msg: String) -> None:
        if self._boat_east is None:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        detections = data.get("detections", [])
        stamp = self._boat_stamp or self.get_clock().now().to_msg()
        self._last_lidar_detections = []
        for d in detections:
            range_m = float(d.get("range", 0.0))
            bearing_rad = float(d.get("bearing", 0.0))
            east, north = self._to_global(range_m, bearing_rad)
            alpha = _normalize_angle((self._boat_heading_rad or 0.0) + bearing_rad)
            g = GlobalDetection()
            g.header.stamp = stamp
            g.header.frame_id = self._map_frame
            g.east = east
            g.north = north
            g.range_m = range_m
            g.bearing_global_rad = alpha
            g.source = "lidar"
            g.class_name = "unknown"
            g.class_id = 255
            g.id = int(d.get("id", 0))
            self._last_lidar_detections.append(g)
        self._publish_combined()

    def _cv_callback(self, msg: String) -> None:
        if self._boat_east is None:
            return
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        detections = data.get("detections", [])
        self.get_logger().info(f"CV callback: {len(detections)} detections received", throttle_duration_sec=2.0)
        stamp = self._boat_stamp or self.get_clock().now().to_msg()
        self._last_cv_detections = []
        for d in detections:
            dist = d.get("distance_m")
            if dist is None:
                class_name = d.get("class_name", "unknown")
                # Dock intentionally has distance_m=None (variable size, CV-only); skip without warning
                if class_name == "dock":
                    self.get_logger().debug("CV dock detection skipped (distance_m=None by design)", throttle_duration_sec=2.0)
                else:
                    self.get_logger().warn(f"CV detection missing distance_m: {class_name}", throttle_duration_sec=2.0)
                continue
            range_m = float(dist)
            bearing_deg = d.get("bearing_deg", 0.0)
            # CV: 0 = forward, + = right. Base_link: 0 = +X forward, +Y = left -> bearing from +X: right positive = -bearing_deg in rad
            bearing_rad = -math.radians(float(bearing_deg))
            east, north = self._to_global(range_m, bearing_rad)
            alpha = _normalize_angle((self._boat_heading_rad or 0.0) + bearing_rad)
            g = GlobalDetection()
            g.header.stamp = stamp
            g.header.frame_id = self._map_frame
            g.east = east
            g.north = north
            g.range_m = range_m
            g.bearing_global_rad = alpha
            g.source = "vision"
            g.class_name = str(d.get("class_name", "unknown"))
            g.class_id = int(d.get("class_id", 255))
            g.id = 0
            self.get_logger().info(f"CV detection added: {g.class_name} at east={east:.2f}, north={north:.2f}, range={range_m:.2f}m", throttle_duration_sec=2.0)
            self._last_cv_detections.append(g)
        if self._last_cv_detections:
            self._publish_combined()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionToGlobalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
