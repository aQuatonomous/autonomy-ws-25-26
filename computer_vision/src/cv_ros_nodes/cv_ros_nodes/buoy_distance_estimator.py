#!/usr/bin/env python3
"""
Distance estimator node for maritime objects.

Reads /camera0/detection_info (final detection output for camera 0) and estimates
distance per bounding box using camera calibration from camera_grid_calibration.
Uses pinhole model: distance = (focal_length * reference_height_m) / height_px.
Uses class-specific reference dimensions based on RoboBoat 2026 specifications.

Subscribes: /camera0/detection_info (std_msgs/String JSON)
Publishes:  /camera0/detection_info_with_distance (std_msgs/String JSON, same structure + estimated_distance_m + reference_height_m per detection)
"""

import json
import os
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def load_calibration(npz_path: str) -> Optional[dict]:
    """
    Load K, dist, image_size from camera_grid_calibration .npz.
    Returns dict with keys: K (3x3), dist, image_size (w, h), or None on failure.
    """
    if not npz_path or not os.path.isfile(npz_path):
        return None
    try:
        data = np.load(npz_path)
        K = data['K']
        dist = data['dist'] if 'dist' in data else np.zeros(5)
        img_size = data['image_size']
        calib_w = int(img_size[0]) if img_size.shape else 640
        calib_h = int(img_size[1]) if img_size.shape and len(img_size) > 1 else 480
        return {
            'K': K,
            'dist': dist,
            'calib_width': calib_w,
            'calib_height': calib_h,
        }
    except Exception:
        return None


class BuoyDistanceEstimatorNode(Node):
    """Estimates distance for each detection from camera0 using grid calibration and class-specific dimensions."""

    def __init__(self):
        super().__init__('maritime_distance_estimator')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter(
            'calibration_file',
            os.path.join(
                os.path.expanduser('~'),
                'autonomy-ws-25-26',
                'computer_vision',
                'camera_grid_calibration',
                'camera_calib.npz',
            ),
        )
        self.declare_parameter('default_reference_height_m', 0.3)

        self._camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        calib_path = self.get_parameter('calibration_file').get_parameter_value().string_value
        self._default_reference_height_m = self.get_parameter('default_reference_height_m').get_parameter_value().double_value

        # Reference dimensions for each class (in meters, primarily height above waterline)
        # Based on RoboBoat 2026 specifications. Small round buoys use 1 ft; pole buoys use 39 in.
        self._reference_dimensions = {
            # Buoys - small round (1 ft) use height_scale 0.5 to correct for bbox including reflection
            'black_buoy': 0.305,        # 1 ft above waterline (Polyform A-2)
            'green_buoy': 0.305,        # 1 ft for gate/small (pole version is green_pole_buoy)
            'green_pole_buoy': 0.991,   # 39 in above waterline (Taylor Made Sur-Mark 950400)
            'red_buoy': 0.305,          # 1 ft for gate/small (pole version is red_pole_buoy)
            'red_pole_buoy': 0.991,     # 39 in above waterline (Taylor Made Sur-Mark 950410)
            'yellow_buoy': 0.305,       # 1 ft above waterline (Polyform A-2)
            
            # Navigation markers - using diamond size as reference for cross/triangle
            'cross': 0.203,             # 8 in diamond size (vertex-to-vertex)
            'triangle': 0.203,          # 8 in diamond size (assumed similar to cross)
            
            # Infrastructure
            'dock': 0.406,              # 16 in height
            
            # Indicator buoys
            'red_indicator_buoy': 0.432,    # 17 in total height
            'green_indicator_buoy': 0.432,  # 17 in total height
            
            # Supply drops - using width since height not specified
            'yellow_supply_drop': 0.406,    # 16.0 in width
            'black_supply_drop': 0.406,     # 16.0 in width
            
            # Docking numbers
            'digit_1': 0.610,          # 24 in banner height
            'digit_2': 0.610,          # 24 in banner height  
            'digit_3': 0.610,          # 24 in banner height
        }

        self._calib = load_calibration(calib_path)
        if self._calib is None:
            self.get_logger().warn(
                f'Calibration not loaded from {calib_path}; distance will not be estimated.'
            )
        else:
            fx = self._calib['K'][0, 0]
            fy = self._calib['K'][1, 1]
            self.get_logger().info(
                f'Loaded calibration: fx={fx:.1f}, fy={fy:.1f}, '
                f'calib size={self._calib["calib_width"]}x{self._calib["calib_height"]}, '
                f'default_reference_height_m={self._default_reference_height_m}, '
                f'loaded {len(self._reference_dimensions)} class-specific dimensions'
            )

        topic_in = f'/camera{self._camera_id}/detection_info'
        topic_out = f'/camera{self._camera_id}/detection_info_with_distance'
        self._sub = self.create_subscription(String, topic_in, self._callback, 10)
        self._pub = self.create_publisher(String, topic_out, 10)
        self.get_logger().info(f'Subscribed to {topic_in}, publishing to {topic_out}')

    def _estimate_distance(self, bbox: list, frame_width: int, frame_height: int, class_name: str = None) -> Optional[float]:
        """
        Pinhole: distance = (fy_eff * reference_height_m) / height_px.
        Uses class-specific reference dimensions when available.
        Scales focal length if frame size differs from calibration size.
        """
        if self._calib is None or not bbox or len(bbox) < 4:
            return None
        
        # Get reference dimension for this class
        reference_height_m = self._reference_dimensions.get(class_name, self._default_reference_height_m)
        
        x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])
        height_px = max(1.0, y2 - y1)
        fy = self._calib['K'][1, 1]
        cw = self._calib['calib_width']
        ch = self._calib['calib_height']
        if cw > 0 and ch > 0 and frame_width > 0 and frame_height > 0:
            fy_eff = fy * (frame_height / ch)
        else:
            fy_eff = fy
        distance_m = (fy_eff * reference_height_m) / height_px
        return distance_m

    def _callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid JSON: {e}')
            return

        frame_width = int(data.get('frame_width', 0))
        frame_height = int(data.get('frame_height', 0))
        detections = data.get('detections', [])
        out_detections = []

        for det in detections:
            det_out = dict(det)
            bbox = det.get('bbox') or [det.get('x1'), det.get('y1'), det.get('x2'), det.get('y2')]
            class_name = det.get('class_name') or det.get('class')  # Try both possible field names
            dist = self._estimate_distance(bbox, frame_width, frame_height, class_name)
            det_out['estimated_distance_m'] = round(dist, 3) if dist is not None else None
            # Add reference dimension info for debugging
            if class_name and class_name in self._reference_dimensions:
                det_out['reference_height_m'] = self._reference_dimensions[class_name]
            else:
                det_out['reference_height_m'] = self._default_reference_height_m
            out_detections.append(det_out)

        data_out = dict(data)
        data_out['detections'] = out_detections
        out_msg = String()
        out_msg.data = json.dumps(data_out)
        self._pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyDistanceEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
