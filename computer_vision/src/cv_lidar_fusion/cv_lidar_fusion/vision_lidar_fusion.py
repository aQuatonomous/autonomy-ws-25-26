#!/usr/bin/env python3
"""
Vision-LiDAR fusion node.

Subscribes to /combined/detection_info (CV) and /tracked_buoys (LiDAR).
Matches by angle: CV provides bearing_deg per detection; LiDAR provides bearing (radians).
Publishes /fused_buoys (TrackedBuoyArray) with color from CV.
"""

import json
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pointcloud_filters.msg import TrackedBuoyArray, TrackedBuoy


# Buoy class_ids from CV (0-5); 6-8 are cross/dock/triangle, not buoys
BUOY_CLASS_IDS = {0, 1, 2, 3, 4, 5}


class VisionLidarFusionNode(Node):
    """Fusion node: match LiDAR tracks to CV detections by bearing; assign color from class_id; publish /fused_buoys."""

    def __init__(self):
        super().__init__('vision_lidar_fusion')

        self.declare_parameter('bearing_threshold', 0.15)
        self._bearing_threshold = float(
            self.get_parameter('bearing_threshold').get_parameter_value().double_value
        )

        self._class_to_color = {
            0: 'black',
            1: 'green',
            2: 'green',
            3: 'red',
            4: 'red',
            5: 'yellow',
        }

        self._latest_cv_detections = []
        self._latest_cv_timestamp = None

        self._cv_sub = self.create_subscription(
            String,
            '/combined/detection_info',
            self._cv_callback,
            10,
        )
        self._lidar_sub = self.create_subscription(
            TrackedBuoyArray,
            '/tracked_buoys',
            self._lidar_callback,
            10,
        )
        self._fused_pub = self.create_publisher(
            TrackedBuoyArray,
            '/fused_buoys',
            10,
        )

        self.get_logger().info(
            f'Vision-LiDAR fusion: bearing_threshold={self._bearing_threshold:.2f} rad. '
            'Subscribed to /combined/detection_info and /tracked_buoys; publishing /fused_buoys.'
        )

    def _cv_callback(self, msg: String) -> None:
        """Parse CV JSON and store detections (combiner provides bearing_deg per detection)."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'CV: invalid JSON: {e}')
            return
        self._latest_cv_detections = data.get('detections', [])
        self._latest_cv_timestamp = data.get('timestamp')

    def _run_fusion(self, msg: TrackedBuoyArray) -> None:
        """
        Match LiDAR tracks to CV detections by bearing; assign color from class_id; publish /fused_buoys.
        CV bearing_deg: 0=forward, negative=left, positive=right. LiDAR bearing: radians, 0=forward, +left.
        Convert: angle_rad = math.radians(bearing_deg), cv_bearing = -angle_rad for comparison with LiDAR.
        """
        out = TrackedBuoyArray()
        out.header = msg.header

        if len(msg.buoys) == 0:
            self._fused_pub.publish(out)
            return

        # Build list of CV detections with valid bearing and buoy class_id; compute cv_bearing (radians, same convention as LiDAR)
        cv_candidates: List[Tuple[float, str, float]] = []
        for d in self._latest_cv_detections:
            cid = d.get('class_id')
            if cid is None or cid not in BUOY_CLASS_IDS:
                continue
            # Combiner publishes bearing_deg (bearing_rad is trimmed from output)
            bearing_deg = d.get('bearing_deg')
            if bearing_deg is None:
                continue
            angle_rad = math.radians(float(bearing_deg))
            cv_bearing = -angle_rad  # match LiDAR convention
            color = self._class_to_color.get(cid, 'unknown')
            score = float(d.get('score', 0.0))
            cv_candidates.append((cv_bearing, color, score))

        # Greedy 1-to-1: for each LiDAR track, pick closest CV detection within threshold; mark used
        used_cv_indices = set()

        for track in msg.buoys:
            fused = TrackedBuoy()
            fused.id = track.id
            fused.range = track.range
            fused.bearing = track.bearing
            fused.x = track.x
            fused.y = track.y
            fused.z_mean = track.z_mean
            fused.confidence = track.confidence
            fused.observation_count = track.observation_count
            fused.first_seen = track.first_seen
            fused.last_seen = track.last_seen

            lidar_bearing = track.bearing
            best_idx = None
            best_diff = self._bearing_threshold + 1.0

            for idx, (cv_bearing, color, score) in enumerate(cv_candidates):
                if idx in used_cv_indices:
                    continue
                diff = abs(cv_bearing - lidar_bearing)
                if diff < best_diff:
                    best_diff = diff
                    best_idx = idx

            if best_idx is not None:
                fused.color = cv_candidates[best_idx][1]
                used_cv_indices.add(best_idx)
            else:
                fused.color = 'unknown'

            out.buoys.append(fused)

        self._fused_pub.publish(out)

    def _lidar_callback(self, msg: TrackedBuoyArray) -> None:
        """On each LiDAR update, run fusion and publish /fused_buoys."""
        self._run_fusion(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionLidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
