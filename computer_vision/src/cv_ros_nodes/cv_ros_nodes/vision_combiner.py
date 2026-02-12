"""
Multi-Camera Detection Combiner ROS2 Node - Per-Camera Bearing Approach

Subscribes to: /camera0/detection_info, /camera1/detection_info, /camera2/detection_info;
  /camera0/task4_detections, /camera1/task4_detections, /camera2/task4_detections
Publishes to: /combined/detection_info (String with all detections; bbox in local camera frame)

PER-CAMERA BEARING APPROACH:
- Each camera detection stays in its local frame (bbox is per-camera coordinates)
- Bearing is computed using:
  1. Camera mounting angle: [-57.7°, 0°, +57.7°] for [left, center, right]
  2. Camera intrinsics derived from 85° horizontal FOV and 1920×1200 resolution
  3. Formula: bearing_deg = mounting_angle + atan2((u - cx), fx)
- Output: boat-relative bearing for each detection (0° = forward, + = right, - = left)

WHAT THIS NODE DOES:
This node combines detections from all 3 side-by-side cameras into a single unified list.
Each detection includes:
- source camera_id (0=left, 1=center, 2=right)
- bbox in local camera frame
- bearing_deg/bearing_rad: angle relative to boat forward direction
- elevation_deg/elevation_rad: angle relative to horizon

HOW IT WORKS:
1. Subscribes to detection_info from all 3 cameras
2. Collects ALL detections from each active camera
3. Computes per-camera bearing using geometry (camera mounting + intrinsics)
4. Adds boat-relative bearing to each detection
5. Publishes combined list at ~30 Hz

OPERATION MODE:
- Latest Value Approach (default): Combines the most recent detection from each camera
- Staleness Filtering: Excludes cameras that haven't updated recently

Usage:
    # Default: Combines all 3 cameras, 1.0s staleness threshold
    python3 vision_combiner.py
    
    # Custom staleness threshold
    python3 vision_combiner.py --staleness_threshold 0.5
    
    # Timestamp-based sync (optional)
    python3 vision_combiner.py --use_timestamp_sync --sync_window 0.05
"""

import math
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import yaml
from typing import Dict, List, Optional
import time

# Camera specifications (Arducam B0495 2.3MP AR0234)
HORIZONTAL_FOV_PER_CAMERA_DEG = 85.0
VERTICAL_FOV_PER_CAMERA_DEG = 69.0
CAMERA_RESOLUTION_WIDTH = 1920
CAMERA_RESOLUTION_HEIGHT = 1200

# Camera mounting angles (boat frame: 0° = forward, + = right, - = left)
# Camera 0 = left, Camera 1 = center, Camera 2 = right
CAMERA_MOUNTING_ANGLES_DEG = [-57.7, 0.0, 57.7]





class DetectionCombiner(Node):
    
    def __init__(self, sync_window: float = 0.05, use_timestamp_sync: bool = False,
                 staleness_threshold: float = 1.0):
        super().__init__('detection_combiner')
        
        self.use_timestamp_sync = use_timestamp_sync
        self.sync_window = sync_window  # Time window for timestamp matching (seconds)
        self.staleness_threshold = staleness_threshold  # Max age for detections (seconds)
        
        # Per-camera frame dimensions: learned from detection_info (inference publishes preprocessed
        # image size, e.g. 1920×1200). Fallback until first message.
        self.frame_width = CAMERA_RESOLUTION_WIDTH
        self.frame_height = CAMERA_RESOLUTION_HEIGHT
        
        # Compute camera intrinsics from FOV and resolution
        self._compute_camera_intrinsics()

        # Store latest detections from each camera (inference)
        self.detections = {
            0: None,  # camera0
            1: None,  # camera1
            2: None   # camera2
        }
        
        self.last_update_times = {
            0: 0.0,
            1: 0.0,
            2: 0.0
        }

        # Task 4 supply-drop detections per camera (from /cameraN/task4_detections)
        self.task4_detections = {0: None, 1: None, 2: None}
        # Indicator buoy detections per camera (from /cameraN/indicator_detections)
        self.indicator_detections = {0: None, 1: None, 2: None}

        # Load class_mapping for class_name resolution
        self._class_id_to_name = self._load_class_mapping()

        # For timestamp-based synchronization: store detection history
        # Format: {timestamp: {camera_id: detection_data}}
        self.detection_history = []
        self.max_history_size = 100  # Keep last 100 detections per camera
        
        # Subscriptions for each camera's detection info
        self._detection_subs = []
        for camera_id in [0, 1, 2]:
            sub = self.create_subscription(
                String,
                f'/camera{camera_id}/detection_info',
                lambda msg, cid=camera_id: self.detection_callback(msg, cid),
                10
            )
            self._detection_subs.append(sub)
            self.get_logger().info(f'Subscribed to: /camera{camera_id}/detection_info')

        for camera_id in [0, 1, 2]:
            sub = self.create_subscription(
                String,
                f'/camera{camera_id}/task4_detections',
                lambda msg, cid=camera_id: self._task4_callback(msg, cid),
                10
            )
            self.get_logger().info(f'Subscribed to: /camera{camera_id}/task4_detections')

        for camera_id in [0, 1, 2]:
            sub = self.create_subscription(
                String,
                f'/camera{camera_id}/indicator_detections',
                lambda msg, cid=camera_id: self._indicator_callback(msg, cid),
                10
            )
            self.get_logger().info(f'Subscribed to: /camera{camera_id}/indicator_detections')

        # Publisher for combined detections
        self.combined_pub = self.create_publisher(
            String,
            '/combined/detection_info',
            10
        )
        
        self.get_logger().info('Detection combiner node initialized')
        self.get_logger().info('Publishing to: /combined/detection_info')
        self.get_logger().info(f'Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')
        self.get_logger().info(f'Camera mounting angles: {CAMERA_MOUNTING_ANGLES_DEG}')
        
        # Timer to periodically publish combined detections (~30 Hz so combined output stays responsive)
        self.timer = self.create_timer(0.033, self.publish_combined)

    def _load_class_mapping(self) -> Dict[int, str]:
        """Load class_id -> class_name from class_mapping.yaml. Returns dict; missing IDs get 'class_N'."""
        paths = []
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('cv_ros_nodes')
            paths.append(os.path.join(pkg_share, 'class_mapping.yaml'))
        except Exception:
            pass
        paths.append(os.path.join(os.path.expanduser('~'), 'autonomy-ws-25-26', 'computer_vision', 'cv_scripts', 'class_mapping.yaml'))
        for path in paths:
            if os.path.isfile(path):
                try:
                    with open(path, 'r') as f:
                        data = yaml.safe_load(f)
                    classes = data.get('classes') or {}
                    return {int(k): str(v) for k, v in classes.items()}
                except Exception as e:
                    self.get_logger().warn(f'Failed to load class_mapping from {path}: {e}')
        self.get_logger().warn('Class mapping not found; detections will have class_name "class_N"')
        return {}

    def _trim_detection_for_output(self, det: Dict) -> Dict:
        """Return a copy of the detection with redundant/radian fields removed for combined output."""
        keys_to_drop = {'x1', 'y1', 'x2', 'y2', 'width', 'height', 'bearing_rad', 'elevation_rad'}
        return {k: v for k, v in det.items() if k not in keys_to_drop}

    def _resolve_class_name(self, det: Dict) -> str:
        """Resolve class_name for a detection. Task4 uses type; others use class_id."""
        if det.get('source') == 'task4' and det.get('type'):
            return det['type']  # yellow_supply_drop, black_supply_drop
        cid = det.get('class_id')
        if cid is not None:
            return self._class_id_to_name.get(int(cid), f'class_{cid}')
        return 'unknown'

    def _compute_camera_intrinsics(self):
        """
        Compute camera intrinsics from FOV and resolution.
        Uses manufacturer specs: 85° horizontal FOV, 69° vertical FOV, 1920×1200 resolution.
        Formula: fx = width / (2 * tan(fov_h / 2))
        """
        # Compute focal lengths from FOV
        self.fx = self.frame_width / (2.0 * math.tan(math.radians(HORIZONTAL_FOV_PER_CAMERA_DEG / 2.0)))
        self.fy = self.frame_height / (2.0 * math.tan(math.radians(VERTICAL_FOV_PER_CAMERA_DEG / 2.0)))
        
        # Principal point at image center (standard assumption)
        self.cx = self.frame_width / 2.0
        self.cy = self.frame_height / 2.0
    
    def _compute_bearing_and_elevation(self, camera_id: int, bbox: List[float]) -> Dict[str, float]:
        """
        Compute boat-relative bearing and elevation from camera detection.
        
        Args:
            camera_id: 0=left, 1=center, 2=right
            bbox: [x1, y1, x2, y2] in camera frame
        
        Returns:
            Dict with bearing_deg, elevation_deg (and optional camera_angle_deg, mounting_angle_deg for debug).
            - bearing: 0° = boat forward, + = right, - = left
            - elevation: 0° = horizon, + = up, - = down
        """
        if len(bbox) != 4:
            return {
                'bearing_deg': 0.0,
                'elevation_deg': 0.0
            }
        
        # Bbox center in camera frame
        u_center = (bbox[0] + bbox[2]) / 2.0
        v_center = (bbox[1] + bbox[3]) / 2.0
        
        # Angle relative to camera optical axis (horizontal)
        camera_angle_rad = math.atan2(u_center - self.cx, self.fx)
        camera_angle_deg = math.degrees(camera_angle_rad)
        
        # Boat-relative bearing: add camera mounting angle
        mounting_angle_deg = CAMERA_MOUNTING_ANGLES_DEG[camera_id]
        bearing_deg = mounting_angle_deg + camera_angle_deg
        
        # Elevation angle (vertical): 0° = horizon, positive = up, negative = down
        elevation_rad = math.atan2(self.cy - v_center, self.fy)
        elevation_deg = math.degrees(elevation_rad)
        
        return {
            'bearing_deg': bearing_deg,
            'elevation_deg': elevation_deg,
            'camera_angle_deg': camera_angle_deg,
            'mounting_angle_deg': mounting_angle_deg
        }
    
    def detection_callback(self, msg: String, camera_id: int):
        """Callback for individual camera detection info"""
        try:
            detection_data = json.loads(msg.data)
            detection_timestamp = detection_data.get('timestamp', time.time())
            
            # Store latest detection (for non-sync mode)
            self.detections[camera_id] = detection_data
            self.last_update_times[camera_id] = time.time()
            
            # Learn frame dimensions from detection_info (bbox already in preprocessed frame)
            if 'frame_width' in detection_data or 'frame_height' in detection_data:
                new_width = int(detection_data.get('frame_width', self.frame_width))
                new_height = int(detection_data.get('frame_height', self.frame_height))
                
                # Recompute intrinsics if frame size changed
                if new_width != self.frame_width or new_height != self.frame_height:
                    self.frame_width = new_width
                    self.frame_height = new_height
                    self._compute_camera_intrinsics()
                    self.get_logger().info(
                        f'Updated frame size to {self.frame_width}×{self.frame_height}, '
                        f'recomputed intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}'
                    )
            
            # If using timestamp sync, try to match with other cameras
            if self.use_timestamp_sync:
                self._try_synchronize_and_publish(detection_timestamp, camera_id, detection_data)
            else:
                # Immediately publish when new data arrives
                self.publish_combined()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detection info from camera{camera_id}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing detection from camera{camera_id}: {e}')

    def _task4_callback(self, msg: String, camera_id: int):
        """Store latest Task 4 supply-drop detections for a camera."""
        try:
            self.task4_detections[camera_id] = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse task4_detections from camera{camera_id}: {e}')

    def _indicator_callback(self, msg: String, camera_id: int):
        """Store latest indicator buoy detections for a camera."""
        try:
            self.indicator_detections[camera_id] = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse indicator_detections from camera{camera_id}: {e}')

    def _indicator_detections_to_combined(self, camera_id: int, indicator_data: dict) -> List[Dict]:
        """
        Convert indicator buoy payload to same format as other detections.
        Uses shape_bbox for bbox (whole buoy); class_id 9=red, 10=green.
        """
        out = []
        for det in indicator_data.get('detections', []):
            bbox = det.get('shape_bbox')
            if not bbox or len(bbox) != 4:
                continue
            det_with_camera = {
                'camera_id': camera_id,
                'class_id': det.get('class_id'),
                'score': det.get('score', 0.0),
                'bbox': list(bbox),
                'indicator_color': det.get('indicator_color'),
                'source': 'indicator_buoy',
            }
            out.append(det_with_camera)
        return out

    def _task4_detections_to_combined(self, camera_id: int, task4_data: dict) -> List[Dict]:
        """
        Convert Task4 payload detections to the same format as inference detections
        so they can be merged into all_detections. shape_bbox is already in preprocessed frame.
        """
        out = []
        for det in task4_data.get('detections', []):
            bbox = det.get('shape_bbox')
            if not bbox or len(bbox) != 4:
                continue
            det_with_camera = {
                'camera_id': camera_id,
                'class_id': det.get('class_id'),
                'score': det.get('score', 0.0),
                'bbox': list(bbox),
                'type': det.get('type'),
                'source': 'task4',
            }
            if det.get('vessel_bbox') is not None:
                det_with_camera['vessel_bbox'] = det['vessel_bbox']
            out.append(det_with_camera)
        return out

    def _try_synchronize_and_publish(self, timestamp: float, source_camera_id: int, detection_data: dict):
        """
        Attempt to synchronize detections from all cameras based on timestamp.
        If detections from all cameras are within sync_window, publish combined output.
        """
        # Check if we have recent detections from other cameras within sync window
        matched_detections = {source_camera_id: detection_data}
        matched_timestamps = {source_camera_id: timestamp}
        
        current_time = time.time()
        
        for camera_id in [0, 1, 2]:
            if camera_id == source_camera_id:
                continue
            
            detection = self.detections[camera_id]
            if detection is None:
                continue
            
            # Check if detection is stale
            time_since_update = current_time - self.last_update_times[camera_id]
            if time_since_update > self.staleness_threshold:
                continue
            
            other_timestamp = detection.get('timestamp', 0.0)
            time_diff = abs(timestamp - other_timestamp)
            
            # If timestamps are close enough, include in synchronized output
            if time_diff <= self.sync_window:
                matched_detections[camera_id] = detection
                matched_timestamps[camera_id] = other_timestamp
        
        # If we have at least 2 cameras synchronized, or if we have all 3, publish
        if len(matched_detections) >= 2:
            self._publish_synchronized(matched_detections, matched_timestamps)
        # Otherwise, if this is the first camera or others are too far off, 
        # we'll wait for timer-based publishing
    
    def _publish_synchronized(self, matched_detections: dict, matched_timestamps: dict):
        """Publish synchronized detections from matched cameras"""
        all_detections = []
        camera_stats = {}
        current_time = time.time()
        
        # Process matched detections (bbox already in preprocessed frame from inference)
        for camera_id, detection_data in matched_detections.items():
            detections = detection_data.get('detections', [])
            for det in detections:
                det_with_camera = det.copy()
                det_with_camera['camera_id'] = camera_id
                # Bbox is already in preprocessed frame (frame_width x frame_height) from inference
                bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
                det_with_camera['bbox'] = list(bbox)
                all_detections.append(det_with_camera)
            
            # Merge Task4 detections for this camera when present
            task4_data = self.task4_detections.get(camera_id)
            if task4_data is not None:
                for det in self._task4_detections_to_combined(camera_id, task4_data):
                    all_detections.append(det)
            # Merge indicator buoy detections for this camera when present
            indicator_data = self.indicator_detections.get(camera_id)
            if indicator_data is not None:
                for det in self._indicator_detections_to_combined(camera_id, indicator_data):
                    all_detections.append(det)

            # Add stats for matched cameras
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('fps', 0.0),
                'timestamp': matched_timestamps[camera_id]
            }
        
        # Compute bearing and elevation for each detection
        for det in all_detections:
            bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            camera_id = det.get('camera_id', 0)
            bearing_info = self._compute_bearing_and_elevation(camera_id, bbox)
            det.update(bearing_info)
        # Add class_name for every detection
        for det in all_detections:
            det['class_name'] = self._resolve_class_name(det)
        # Trim output: drop x1,y1,x2,y2,width,height and radian fields
        all_detections = [self._trim_detection_for_output(d) for d in all_detections]

        # Add stats for unmatched cameras
        for camera_id in [0, 1, 2]:
            if camera_id not in matched_detections:
                time_since_update = current_time - self.last_update_times[camera_id]
                if self.detections[camera_id] is None:
                    camera_stats[camera_id] = {'status': 'no_data', 'num_detections': 0}
                elif time_since_update > self.staleness_threshold:
                    camera_stats[camera_id] = {
                        'status': 'stale',
                        'num_detections': 0,
                        'time_since_update': time_since_update
                    }
                else:
                    # Camera has data but timestamp doesn't match
                    camera_stats[camera_id] = {
                        'status': 'unsynchronized',
                        'num_detections': 0,
                        'time_since_update': time_since_update
                    }
        
        # Calculate average timestamp of matched detections
        avg_timestamp = sum(matched_timestamps.values()) / len(matched_timestamps) if matched_timestamps else current_time
        
        # Create combined detection info
        combined_info = {
            'timestamp': avg_timestamp,
            'num_cameras': 3,
            'num_synchronized_cameras': len(matched_detections),
            'total_detections': len(all_detections),
            'camera_stats': camera_stats,
            'detections': all_detections,
            'synchronized': True,
            'camera_info': {
                'frame_width': self.frame_width,
                'frame_height': self.frame_height,
                'horizontal_fov_deg': HORIZONTAL_FOV_PER_CAMERA_DEG,
                'vertical_fov_deg': VERTICAL_FOV_PER_CAMERA_DEG,
                'mounting_angles_deg': CAMERA_MOUNTING_ANGLES_DEG,
                'fx': self.fx,
                'fy': self.fy,
                'cx': self.cx,
                'cy': self.cy
            }
        }
        
        # Publish combined detection info
        msg = String()
        msg.data = json.dumps(combined_info)
        self.combined_pub.publish(msg)
    
    def publish_combined(self):
        """
        Combine detections from all 3 cameras and publish.
        Uses latest value approach with staleness filtering.
        Only includes detections from cameras that have updated recently.
        
        This is the main function that combines all camera views:
        - Loops through all 3 cameras (0, 1, 2)
        - Collects ALL detections from each active camera
        - Adds camera_id to each detection
        - Publishes unified list
        """
        current_time = time.time()
        all_detections = []
        camera_stats = {}
        active_cameras = 0
        stale_cameras = 0
        no_data_cameras = 0
        
        # Collect detections from all 3 cameras (side-by-side configuration)
        for camera_id in [0, 1, 2]:
            detection_data = self.detections[camera_id]
            
            # Check if camera has no data at all
            if detection_data is None:
                no_data_cameras += 1
                camera_stats[camera_id] = {
                    'status': 'no_data',
                    'num_detections': 0,
                    'time_since_update': None
                }
                continue
            
            # Check if detection is stale (too old)
            time_since_update = current_time - self.last_update_times[camera_id]
            is_stale = time_since_update > self.staleness_threshold
            
            if is_stale:
                stale_cameras += 1
                camera_stats[camera_id] = {
                    'status': 'stale',
                    'num_detections': 0,
                    'time_since_update': round(time_since_update, 3),
                    'staleness_threshold': self.staleness_threshold,
                    'fps': detection_data.get('fps', 0.0),
                    'last_timestamp': detection_data.get('timestamp', 0.0)
                }
                # Exclude stale camera from output - don't add its detections
                self.get_logger().warn(
                    f'Camera{camera_id} is stale: {time_since_update:.3f}s since last update '
                    f'(threshold: {self.staleness_threshold}s) - excluding from output'
                )
                continue
            
            # Camera is active and fresh - include ALL its detections (bbox already in preprocessed frame from inference)
            active_cameras += 1
            detections = detection_data.get('detections', [])
            for det in detections:
                det_with_camera = det.copy()
                det_with_camera['camera_id'] = camera_id  # Tag each detection with source camera
                # Bbox is already in preprocessed frame (frame_width x frame_height) from inference
                bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
                det_with_camera['bbox'] = list(bbox)
                all_detections.append(det_with_camera)  # Add to combined list
            
            # Add stats for active camera
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('fps', 0.0),
                'timestamp': detection_data.get('timestamp', 0.0),
                'time_since_update': round(time_since_update, 3)
            }
        
        # Merge Task4 supply-drop detections when present
        for camera_id in [0, 1, 2]:
            task4_data = self.task4_detections[camera_id]
            if task4_data is not None:
                for det in self._task4_detections_to_combined(camera_id, task4_data):
                    all_detections.append(det)
        # Merge indicator buoy detections when present
        for camera_id in [0, 1, 2]:
            indicator_data = self.indicator_detections[camera_id]
            if indicator_data is not None:
                for det in self._indicator_detections_to_combined(camera_id, indicator_data):
                    all_detections.append(det)

        # Compute bearing and elevation for each detection
        for det in all_detections:
            bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            camera_id = det.get('camera_id', 0)
            bearing_info = self._compute_bearing_and_elevation(camera_id, bbox)
            det.update(bearing_info)
        # Add class_name for every detection
        for det in all_detections:
            det['class_name'] = self._resolve_class_name(det)
        # Trim output: drop x1,y1,x2,y2,width,height and radian fields
        all_detections = [self._trim_detection_for_output(d) for d in all_detections]

        # Create combined detection info
        combined_info = {
            'timestamp': current_time,
            'num_cameras': 3,
            'num_active_cameras': active_cameras,
            'num_stale_cameras': stale_cameras,
            'num_no_data_cameras': no_data_cameras,
            'staleness_threshold': self.staleness_threshold,
            'total_detections': len(all_detections),
            'camera_stats': camera_stats,
            'detections': all_detections,
            'synchronized': self.use_timestamp_sync if self.use_timestamp_sync else None,
            'camera_info': {
                'frame_width': self.frame_width,
                'frame_height': self.frame_height,
                'horizontal_fov_deg': HORIZONTAL_FOV_PER_CAMERA_DEG,
                'vertical_fov_deg': VERTICAL_FOV_PER_CAMERA_DEG,
                'mounting_angles_deg': CAMERA_MOUNTING_ANGLES_DEG,
                'fx': self.fx,
                'fy': self.fy,
                'cx': self.cx,
                'cy': self.cy
            }
        }
        
        # Publish combined detection info
        msg = String()
        msg.data = json.dumps(combined_info)
        self.combined_pub.publish(msg)
        
        # Log health status periodically
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 30 == 0:  # Log every ~1 s
            if stale_cameras > 0 or no_data_cameras > 0:
                self.get_logger().warn(
                    f'Camera health: {active_cameras} active, {stale_cameras} stale, '
                    f'{no_data_cameras} no data | Total detections: {len(all_detections)}'
                )
            else:
                # Ensure all cameras have stats before logging
                cam0_detections = camera_stats.get(0, {}).get('num_detections', 0)
                cam1_detections = camera_stats.get(1, {}).get('num_detections', 0)
                cam2_detections = camera_stats.get(2, {}).get('num_detections', 0)
                self.get_logger().info(
                    f'Combined detections: {len(all_detections)} total | '
                    f'Active cameras: {active_cameras}/3 | '
                    f'Camera0: {cam0_detections}, '
                    f'Camera1: {cam1_detections}, '
                    f'Camera2: {cam2_detections}'
                )


def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser(description='ROS2 Multi-Camera Detection Combiner')
    parser.add_argument('--staleness_threshold', type=float, default=1.0,
                       help='Maximum age (seconds) for detections before excluding camera (default: 1.0s)')
    parser.add_argument('--sync_window', type=float, default=0.05,
                       help='Time window (seconds) for timestamp-based synchronization (default: 0.05 = 50ms)')
    parser.add_argument('--use_timestamp_sync', action='store_true',
                       help='Enable timestamp-based synchronization (default: latest value approach)')
    args_parsed, _ = parser.parse_known_args(args=args)
    
    node = DetectionCombiner(
        sync_window=args_parsed.sync_window,
        use_timestamp_sync=args_parsed.use_timestamp_sync,
        staleness_threshold=args_parsed.staleness_threshold
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
