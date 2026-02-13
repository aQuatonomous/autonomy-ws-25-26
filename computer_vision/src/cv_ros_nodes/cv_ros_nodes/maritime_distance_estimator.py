#!/usr/bin/env python3
"""
Maritime Distance Estimator Node - Specs-Based Approach

Subscribes to /combined/detection_info and estimates distance for each detection using
camera specifications (AR0234, 3.56mm focal length, 3μm pixels, 1920×1200) instead
of calibration files. Uses class-specific reference dimensions from RoboBoat 2026 specs.

This replaces the old buoy_distance_estimator.py with a more accurate specs-based approach
that works consistently across different capture methods and doesn't depend on calibration files.

Subscribes: /combined/detection_info (combined detections from all cameras)
Publishes:  /combined/detection_info_with_distance (same structure + distance_m field per detection)

Camera Specifications (AR0234, Arducam B0495):
- Sensor: 1920(H) × 1200(V) pixels
- Pixel size: 3 μm × 3 μm  
- Focal length: 3.56 mm
- FOV: 98°(D) × 85°(H) × 69°(V)
- Resolution: 1920×1200@50fps (USB 3.0)

Distance Formula: distance_m = (fy * reference_height_m) / height_px
where fy is computed from specs: fy = focal_length_mm / pixel_size_mm

Reference dimensions from RoboBoat 2026 specifications:
- Small buoys (1 ft above waterline): black_buoy, green_buoy, red_buoy, yellow_buoy  
- Pole buoys (39 in above waterline): green_pole_buoy, red_pole_buoy
- Dock (16 in height), digits (24 in banners), etc.
"""

import json
import os
from typing import Optional, Dict, List

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# AR0234 Camera Specifications
PIXEL_SIZE_UM = 3.0                    # 3 μm × 3 μm square pixels
FOCAL_LENGTH_MM = 3.56                 # 3.56 mm focal length
SENSOR_WIDTH_PX = 1920                 # 1920×1200 active area
SENSOR_HEIGHT_PX = 1200
HORIZONTAL_FOV_DEG = 85.0              # From manufacturer specs
VERTICAL_FOV_DEG = 69.0

# Computed focal length in pixels: f_px = f_mm / (pixel_pitch_mm)
FY_PX = FOCAL_LENGTH_MM / (PIXEL_SIZE_UM / 1000.0)  # ~1186.67 pixels
FX_PX = FY_PX  # Square pixels
CX_PX = SENSOR_WIDTH_PX / 2.0
CY_PX = SENSOR_HEIGHT_PX / 2.0

# K matrix for native 1920×1200 resolution (scaled for other resolutions)
K_NATIVE = np.array([
    [FX_PX, 0, CX_PX],
    [0, FY_PX, CY_PX],
    [0, 0, 1.0],
], dtype=np.float64)

# Reference dimensions (meters) - RoboBoat 2026 specifications
# Based on height above waterline for consistent distance measurement
REFERENCE_DIMENSIONS = {
    # Small buoys (1 ft above waterline) - Polyform A-0/A-2
    'black_buoy': 0.305,        # 1 ft above waterline
    'green_buoy': 0.305,        # 1 ft for gate/marking buoys
    'red_buoy': 0.305,          # 1 ft for gate/marking buoys  
    'yellow_buoy': 0.305,       # 1 ft above waterline (Polyform A-2)
    
    # Pole buoys (39 in above waterline) - Taylor Made Sur-Mark
    'green_pole_buoy': 0.991,   # 39 in above waterline (950400)
    'red_pole_buoy': 0.991,     # 39 in above waterline (950410)
    
    # Navigation markers - using diamond reference size
    'cross': 0.203,             # 8 in diamond size (vertex-to-vertex)
    'triangle': 0.203,          # 8 in diamond size (estimated similar)
    
    # Infrastructure and targets
    'dock': 0.406,              # 16 in height
    'red_indicator_buoy': 0.432,    # 17 in total height
    'green_indicator_buoy': 0.432,  # 17 in total height
    'yellow_supply_drop': 0.406,    # 16.0 in width (height not specified)
    'black_supply_drop': 0.406,     # 16.0 in width (height not specified)
    
    # Docking number banners
    'digit_1': 0.610,          # 24 in × 24 in banner
    'digit_2': 0.610,          # 24 in × 24 in banner
    'digit_3': 0.610,          # 24 in × 24 in banner
}
DEFAULT_REFERENCE_M = 0.305  # Default to 1 ft for unknown objects


class MaritimeDistanceEstimatorNode(Node):
    """
    Estimates distance for combined detections using AR0234 camera specifications.
    
    Uses pinhole model with camera intrinsics computed from manufacturer specs
    rather than calibration files. More accurate and consistent across different
    capture methods (video extraction, photo apps, terminal capture, etc.).
    """

    def __init__(self):
        super().__init__('maritime_distance_estimator')
        
        # Camera intrinsics from AR0234 specifications
        self._native_fy = FY_PX
        self._native_fx = FX_PX
        self._native_width = SENSOR_WIDTH_PX
        self._native_height = SENSOR_HEIGHT_PX
        
        # Current frame dimensions (updated from combined detection messages)
        self._frame_width = SENSOR_WIDTH_PX
        self._frame_height = SENSOR_HEIGHT_PX
        
        self.get_logger().info(
            f'Maritime Distance Estimator initialized'
        )
        self.get_logger().info(
            f'Camera specs: {SENSOR_WIDTH_PX}×{SENSOR_HEIGHT_PX}, {PIXEL_SIZE_UM}μm pixels, {FOCAL_LENGTH_MM}mm focal length'
        )
        self.get_logger().info(
            f'Native fy: {self._native_fy:.1f} px, loaded {len(REFERENCE_DIMENSIONS)} object reference dimensions'
        )

        # Subscribe to combined detections, publish with distance estimates
        self._sub = self.create_subscription(
            String, 
            '/combined/detection_info', 
            self._callback, 
            10
        )
        self._pub = self.create_publisher(
            String, 
            '/combined/detection_info_with_distance', 
            10
        )
        
        self.get_logger().info('Subscribed to: /combined/detection_info')
        self.get_logger().info('Publishing to: /combined/detection_info_with_distance')

    def _get_effective_fy(self, frame_width: int, frame_height: int) -> float:
        """
        Compute effective fy for given frame dimensions.
        Scales native focal length if frame size differs from 1920×1200.
        """
        if frame_width == self._native_width and frame_height == self._native_height:
            return self._native_fy
        
        # Scale fy based on height ratio (focal length scales with resolution)
        scale_y = frame_height / self._native_height
        return self._native_fy * scale_y

    def _estimate_distance_m(self, detection: Dict) -> Optional[float]:
        """
        Estimate distance for a single detection using pinhole model.
        
        Formula: distance = (fy_eff * reference_height_m) / height_px
        
        Args:
            detection: Detection dict with bbox, class_name, camera_id
            
        Returns:
            Distance in meters, or None if calculation fails
        """
        # Extract bbox and validate
        bbox = detection.get('bbox')
        if not bbox or len(bbox) < 4:
            return None
            
        class_name = detection.get('class_name', '')
        x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])
        height_px = max(1.0, y2 - y1)
        
        # Get reference dimension for this object class
        reference_height_m = REFERENCE_DIMENSIONS.get(class_name, DEFAULT_REFERENCE_M)
        
        # Compute effective focal length for current frame size
        fy_eff = self._get_effective_fy(self._frame_width, self._frame_height)
        
        # Distance calculation: pinhole camera model
        distance_m = (fy_eff * reference_height_m) / height_px
        
        return distance_m

    def _callback(self, msg: String) -> None:
        """
        Process combined detection message and add distance estimates.
        
        Reads combined detections, estimates distance for each detection,
        and republishes with distance_m field added to each detection.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid JSON in combined detections: {e}')
            return

        # Update frame dimensions from camera_info if available
        camera_info = data.get('camera_info', {})
        if 'frame_width' in camera_info and 'frame_height' in camera_info:
            new_width = int(camera_info['frame_width'])
            new_height = int(camera_info['frame_height'])
            if new_width != self._frame_width or new_height != self._frame_height:
                self._frame_width = new_width
                self._frame_height = new_height
                fy_eff = self._get_effective_fy(new_width, new_height)
                self.get_logger().info(
                    f'Updated frame size to {new_width}×{new_height}, fy_eff: {fy_eff:.1f} px'
                )

        # Process detections and add distance estimates
        detections = data.get('detections', [])
        enhanced_detections = []
        
        for detection in detections:
            # Copy detection and add distance estimate
            enhanced_det = dict(detection)
            distance_m = self._estimate_distance_m(detection)
            
            if distance_m is not None:
                enhanced_det['distance_m'] = round(distance_m, 2)
                enhanced_det['distance_ft'] = round(distance_m * 3.28084, 1)
                # Add reference dimension used for transparency/debugging
                class_name = detection.get('class_name', '')
                enhanced_det['reference_height_m'] = REFERENCE_DIMENSIONS.get(class_name, DEFAULT_REFERENCE_M)
            else:
                enhanced_det['distance_m'] = None
                enhanced_det['distance_ft'] = None
                enhanced_det['reference_height_m'] = None
            
            enhanced_detections.append(enhanced_det)

        # Update data with enhanced detections
        enhanced_data = dict(data)
        enhanced_data['detections'] = enhanced_detections
        enhanced_data['distance_estimation'] = {
            'method': 'specs_based',
            'camera_model': 'AR0234',
            'focal_length_mm': FOCAL_LENGTH_MM,
            'pixel_size_um': PIXEL_SIZE_UM,
            'native_resolution': f'{SENSOR_WIDTH_PX}×{SENSOR_HEIGHT_PX}',
            'current_resolution': f'{self._frame_width}×{self._frame_height}',
            'effective_fy_px': round(self._get_effective_fy(self._frame_width, self._frame_height), 1),
            'num_object_references': len(REFERENCE_DIMENSIONS)
        }

        # Publish enhanced detections
        out_msg = String()
        out_msg.data = json.dumps(enhanced_data)
        self._pub.publish(out_msg)

        # Log summary
        num_with_distance = sum(1 for d in enhanced_detections if d.get('distance_m') is not None)
        if enhanced_detections:
            self.get_logger().debug(
                f'Processed {len(enhanced_detections)} detections, {num_with_distance} with distance estimates'
            )


def main(args=None):
    """Main entry point for maritime distance estimator node."""
    rclpy.init(args=args)
    node = MaritimeDistanceEstimatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Maritime distance estimator shutting down...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()