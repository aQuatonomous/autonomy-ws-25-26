"""
Multi-Camera Detection Combiner ROS2 Node

Subscribes to: /camera0/detection_info, /camera1/detection_info, /camera2/detection_info;
  /camera0/task4_detections, /camera1/task4_detections, /camera2/task4_detections
Publishes to: /combined/detection_info (String with all detections; bbox in global frame; see GLOBAL FRAME SPEC below)

GLOBAL FRAME SPEC:
- Input: Per-camera frame_width and frame_height (from detection_info; preprocessed image size, e.g. 1920×1200).
- Global frame (stitched panorama): width = 3 * frame_width, height = frame_height (e.g. 5760×1200).
  Origin (0,0) top-left; x increases left-to-right across the three cameras (camera 0 left, 1 center, 2 right).
- Conversion: bbox in camera N (local [x1,y1,x2,y2]) → global bbox: x_global = x_local + N * frame_width, y unchanged.
- Published detections: each has "bbox" in global frame [x1,y1,x2,y2], and "global_frame": {"width": W, "height": H}.
- Effective global frame (angle-linear): width = raw_width * (225/255) = 3×frame_width − 2×overlap in angle terms;
  height = frame_height. So e.g. 5760×1200 → effective width ≈ 5082, height 1200.
- Angles (per detection): angle_deg/angle_rad = horizontal bearing (0° = center, negative = left, positive = right),
  over effective span 225° (3×85° FOV − 2×15° overlap). elevation_deg/elevation_rad = vertical (0° = horizon, + up, − down),
  over ±34.5° (69° vertical FOV). effective_global_frame in payload documents these spans and bounds.

WHAT THIS NODE DOES:
This node combines detections from all 3 side-by-side cameras into a single unified list.
Each detection includes its source camera_id (0, 1, or 2) so you know which camera saw it.

HOW IT WORKS:
1. Subscribes to detection_info from all 3 cameras
2. Collects ALL detections from each active camera
3. Adds camera_id to each detection
4. Publishes combined list at ~15 Hz (matches camera rate)

OPERATION MODE:
- Latest Value Approach (default): Combines the most recent detection from each camera,
  regardless of when they were captured. This is fast and simple.
  
- Staleness Filtering: Automatically excludes detections from cameras that haven't
  updated recently. If a camera's latest detection is older than the staleness threshold,
  it's excluded from the output (indicating the camera may have failed or stalled).

DEDUPLICATION:
When --deduplicate_overlap is enabled, detections are merged by global distance: same class_id
and centers within --dedup_global_distance fraction of frame width (default 0.1). After converting
to global coordinates, duplicates in overlap regions are merged into one object (higher score kept).

Usage:
    # Default: Combines all 3 cameras, 1.0s staleness threshold, NO deduplication
    python3 vision_combiner.py
    
    # Enable deduplication by global distance (recommended for side-by-side cameras)
    python3 vision_combiner.py --deduplicate_overlap --dedup_global_distance 0.1
    
    # Custom staleness threshold (0.5s - more strict)
    python3 vision_combiner.py --staleness_threshold 0.5
    
    # Very lenient (2.0s - allows slower cameras)
    python3 vision_combiner.py --staleness_threshold 2.0
    
    # Optional: Timestamp-based sync mode (more complex)
    python3 vision_combiner.py --use_timestamp_sync --sync_window 0.05
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict, List, Optional
import time

# Effective global frame: 85° FOV per camera, 15° overlap per boundary
HORIZONTAL_FOV_PER_CAMERA_DEG = 85
VERTICAL_FOV_PER_CAMERA_DEG = 69
OVERLAP_DEG = 15
TOTAL_HORIZONTAL_ANGLE_DEG = 255  # 3 * 85 (full span; effective width = raw_width * 225/255)
EFFECTIVE_HORIZONTAL_ANGLE_SPAN_DEG = 225  # 3*85 - 2*15 (unique span after overlap)
EFFECTIVE_HORIZONTAL_ANGLE_LEFT_DEG = -112.5
EFFECTIVE_HORIZONTAL_ANGLE_RIGHT_DEG = 112.5
EFFECTIVE_VERTICAL_ANGLE_SPAN_DEG = 69
EFFECTIVE_VERTICAL_ANGLE_DOWN_DEG = -34.5
EFFECTIVE_VERTICAL_ANGLE_UP_DEG = 34.5





class DetectionCombiner(Node):
    
    def __init__(self, sync_window: float = 0.05, use_timestamp_sync: bool = False,
                 staleness_threshold: float = 1.0, deduplicate_overlap: bool = False,
                 dedup_global_distance: float = 0.1):
        super().__init__('detection_combiner')
        
        self.use_timestamp_sync = use_timestamp_sync
        self.sync_window = sync_window  # Time window for timestamp matching (seconds)
        self.staleness_threshold = staleness_threshold  # Max age for detections (seconds)
        self.deduplicate_overlap = deduplicate_overlap  # Enable dedup by global distance when True
        self.dedup_global_distance = dedup_global_distance  # Fraction of frame width; merge if centers within this
        
        # Per-camera frame dimensions: learned from detection_info (inference publishes preprocessed
        # image size, e.g. 1920×1200). Fallback until first message so global_* and angles are defined.
        self.frame_width = 640
        self.frame_height = 480
        # Global frame: 3 cameras side-by-side → width = 3 * frame_width, height = frame_height
        self.global_width = 3 * self.frame_width
        self.global_height = self.frame_height

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
        
        # Publisher for combined detections
        self.combined_pub = self.create_publisher(
            String,
            '/combined/detection_info',
            10
        )
        
        self.get_logger().info('Detection combiner node initialized')
        self.get_logger().info('Publishing to: /combined/detection_info')
        
        # Timer to periodically publish combined detections (~30 Hz so combined output stays responsive)
        self.timer = self.create_timer(0.033, self.publish_combined)
    
    def detection_callback(self, msg: String, camera_id: int):
        """Callback for individual camera detection info"""
        try:
            detection_data = json.loads(msg.data)
            detection_timestamp = detection_data.get('timestamp', time.time())
            
            # Store latest detection (for non-sync mode)
            self.detections[camera_id] = detection_data
            self.last_update_times[camera_id] = time.time()
            
            # Learn frame dimensions from detection_info (bbox already in preprocessed frame)
            if 'frame_width' in detection_data:
                self.frame_width = int(detection_data['frame_width'])
            if 'frame_height' in detection_data:
                self.frame_height = int(detection_data['frame_height'])
            self.global_width = 3 * self.frame_width
            self.global_height = self.frame_height
            
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

    def _task4_detections_to_combined(self, camera_id: int, task4_data: dict) -> List[Dict]:
        """
        Convert Task4 payload detections to the same format as inference detections
        so they can be merged into all_detections. shape_bbox is already in preprocessed frame (640x480).
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
                'x1': bbox[0], 'y1': bbox[1], 'x2': bbox[2], 'y2': bbox[3],
                'width': bbox[2] - bbox[0],
                'height': bbox[3] - bbox[1],
                'type': det.get('type'),
                'source': 'task4',
            }
            if det.get('vessel_bbox') is not None:
                det_with_camera['vessel_bbox'] = det['vessel_bbox']
            out.append(det_with_camera)
        return out

    def _to_global_bbox(self, bbox: List[float], camera_id: int) -> List[float]:
        """Transform bbox from camera-local (frame_width x frame_height) to global. x += camera_id*frame_width."""
        if len(bbox) != 4:
            return bbox
        x1, y1, x2, y2 = bbox
        o = camera_id * self.frame_width
        return [x1 + o, y1, x2 + o, y2]

    def _to_effective_global(self, camera_id: int, bbox: List[float]) -> Dict[str, float]:
        """
        Convert detection center (camera_id, bbox in per-camera coords) to effective global frame.
        Returns effective_x, effective_y (pixels), angle_deg (0=center, negative=left, positive=right), angle_rad,
        elevation_deg (0=horizon, positive=up, negative=down), elevation_rad.
        """
        if len(bbox) != 4:
            return {
                'effective_x': 0.0,
                'effective_y': 0.0,
                'angle_deg': 0.0,
                'angle_rad': 0.0,
                'elevation_deg': 0.0,
                'elevation_rad': 0.0,
            }
        raw_width = 3 * self.frame_width
        effective_width = round(raw_width * EFFECTIVE_HORIZONTAL_ANGLE_SPAN_DEG / TOTAL_HORIZONTAL_ANGLE_DEG)
        x_center = (float(bbox[0]) + float(bbox[2])) / 2.0
        y_center = (float(bbox[1]) + float(bbox[3])) / 2.0
        x_raw = camera_id * self.frame_width + x_center
        effective_x = x_raw * effective_width / raw_width
        effective_y = y_center
        angle_deg = EFFECTIVE_HORIZONTAL_ANGLE_LEFT_DEG + (x_raw / raw_width) * EFFECTIVE_HORIZONTAL_ANGLE_SPAN_DEG
        angle_rad = math.radians(angle_deg)
        # Elevation: 0 = horizon, + up, - down. y=0 top -> +34.5°, y=frame_height/2 -> 0°, y=frame_height -> -34.5°
        elevation_deg = EFFECTIVE_VERTICAL_ANGLE_UP_DEG + (y_center / self.frame_height) * (
            EFFECTIVE_VERTICAL_ANGLE_DOWN_DEG - EFFECTIVE_VERTICAL_ANGLE_UP_DEG
        )
        elevation_rad = math.radians(elevation_deg)
        return {
            'effective_x': effective_x,
            'effective_y': effective_y,
            'angle_deg': angle_deg,
            'angle_rad': angle_rad,
            'elevation_deg': elevation_deg,
            'elevation_rad': elevation_rad,
        }

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
    
    def _deduplicate_global_distance(self, all_detections: List[Dict], max_distance_px: float) -> List[Dict]:
        """
        Merge detections that have the same class_id and whose global centers are within max_distance_px.
        Keeps the detection with higher confidence score per cluster.
        Expects each detection to have global_center_x and global_center_y set.
        """
        if not all_detections or max_distance_px <= 0:
            return all_detections
        sorted_detections = sorted(all_detections, key=lambda x: x.get('score', 0.0), reverse=True)
        kept = []
        removed_count = 0
        for current_det in sorted_detections:
            gx = current_det.get('global_center_x')
            gy = current_det.get('global_center_y')
            cid = current_det.get('class_id')
            if gx is None or gy is None:
                kept.append(current_det)
                continue
            is_duplicate = False
            for kept_det in kept:
                if kept_det.get('class_id') != cid:
                    continue
                kx = kept_det.get('global_center_x')
                ky = kept_det.get('global_center_y')
                if kx is None or ky is None:
                    continue
                dist = math.sqrt((gx - kx) ** 2 + (gy - ky) ** 2)
                if dist <= max_distance_px:
                    is_duplicate = True
                    removed_count += 1
                    break
            if not is_duplicate:
                kept.append(current_det)
        if removed_count > 0:
            self.get_logger().debug(f'Global distance deduplication: removed {removed_count} duplicate(s)')
        return kept
    
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
            
            # Add stats for matched cameras
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('fps', 0.0),
                'timestamp': matched_timestamps[camera_id]
            }
        
        # Add global bbox and center to each detection for dedup and output
        for det in all_detections:
            bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            cid = det.get('camera_id', 0)
            gbbox = self._to_global_bbox(bbox, cid)
            det['global_bbox'] = gbbox
            det['global_center_x'] = (gbbox[0] + gbbox[2]) / 2.0
            det['global_center_y'] = (gbbox[1] + gbbox[3]) / 2.0
        
        # Deduplicate by global distance when enabled (same class_id, centers within threshold)
        if self.deduplicate_overlap and len(all_detections) > 0:
            max_distance_px = self.frame_width * self.dedup_global_distance
            all_detections = self._deduplicate_global_distance(all_detections, max_distance_px)
        
        # Per-detection effective coords, angle and elevation (use per-camera bbox for angle computation)
        for det in all_detections:
            bbox_local = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            eff = self._to_effective_global(det.get('camera_id', 0), bbox_local)
            det['effective_x'] = eff['effective_x']
            det['effective_y'] = eff['effective_y']
            det['angle_deg'] = eff['angle_deg']
            det['angle_rad'] = eff['angle_rad']
            det['elevation_deg'] = eff['elevation_deg']
            det['elevation_rad'] = eff['elevation_rad']
            det['bbox'] = det['global_bbox']  # Publish bbox in global frame
        effective_width = int(round((3 * self.frame_width) * EFFECTIVE_HORIZONTAL_ANGLE_SPAN_DEG / TOTAL_HORIZONTAL_ANGLE_DEG))
        effective_height = self.frame_height
        effective_global_frame = {
            'width': effective_width,
            'height': effective_height,
            'effective_angle_span_deg': EFFECTIVE_HORIZONTAL_ANGLE_SPAN_DEG,
            'effective_angle_left_deg': EFFECTIVE_HORIZONTAL_ANGLE_LEFT_DEG,
            'effective_angle_right_deg': EFFECTIVE_HORIZONTAL_ANGLE_RIGHT_DEG,
            'effective_vertical_angle_span_deg': EFFECTIVE_VERTICAL_ANGLE_SPAN_DEG,
            'effective_vertical_angle_down_deg': EFFECTIVE_VERTICAL_ANGLE_DOWN_DEG,
            'effective_vertical_angle_up_deg': EFFECTIVE_VERTICAL_ANGLE_UP_DEG,
        }
        
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
            'global_frame': {'width': self.global_width, 'height': self.global_height},
            'effective_global_frame': effective_global_frame,
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
        
        # Add global bbox and center to each detection for dedup and output
        for det in all_detections:
            bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            cid = det.get('camera_id', 0)
            gbbox = self._to_global_bbox(bbox, cid)
            det['global_bbox'] = gbbox
            det['global_center_x'] = (gbbox[0] + gbbox[2]) / 2.0
            det['global_center_y'] = (gbbox[1] + gbbox[3]) / 2.0
        
        # Deduplicate by global distance when enabled (same class_id, centers within threshold)
        if self.deduplicate_overlap and len(all_detections) > 0:
            max_distance_px = self.frame_width * self.dedup_global_distance
            all_detections = self._deduplicate_global_distance(all_detections, max_distance_px)
        
        # Per-detection effective coords, angle and elevation; publish bbox in global frame
        effective_width = int(round((3 * self.frame_width) * EFFECTIVE_HORIZONTAL_ANGLE_SPAN_DEG / TOTAL_HORIZONTAL_ANGLE_DEG))
        effective_height = self.frame_height
        effective_global_frame = {
            'width': effective_width,
            'height': effective_height,
            'effective_angle_span_deg': EFFECTIVE_HORIZONTAL_ANGLE_SPAN_DEG,
            'effective_angle_left_deg': EFFECTIVE_HORIZONTAL_ANGLE_LEFT_DEG,
            'effective_angle_right_deg': EFFECTIVE_HORIZONTAL_ANGLE_RIGHT_DEG,
            'effective_vertical_angle_span_deg': EFFECTIVE_VERTICAL_ANGLE_SPAN_DEG,
            'effective_vertical_angle_down_deg': EFFECTIVE_VERTICAL_ANGLE_DOWN_DEG,
            'effective_vertical_angle_up_deg': EFFECTIVE_VERTICAL_ANGLE_UP_DEG,
        }
        for det in all_detections:
            bbox_local = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
            eff = self._to_effective_global(det.get('camera_id', 0), bbox_local)
            det['effective_x'] = eff['effective_x']
            det['effective_y'] = eff['effective_y']
            det['angle_deg'] = eff['angle_deg']
            det['angle_rad'] = eff['angle_rad']
            det['elevation_deg'] = eff['elevation_deg']
            det['elevation_rad'] = eff['elevation_rad']
            det['bbox'] = det['global_bbox']  # Publish bbox in global frame
        
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
            'global_frame': {'width': self.global_width, 'height': self.global_height},
            'effective_global_frame': effective_global_frame,
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
    parser.add_argument('--deduplicate_overlap', action='store_true',
                       help='Enable deduplication by global distance for side-by-side cameras (default: disabled)')
    parser.add_argument('--dedup_global_distance', type=float, default=0.1,
                       help='When deduplicate_overlap enabled: merge detections with same class_id and centers within this fraction of frame width (default: 0.1)')
    args_parsed, _ = parser.parse_known_args(args=args)
    
    node = DetectionCombiner(
        sync_window=args_parsed.sync_window,
        use_timestamp_sync=args_parsed.use_timestamp_sync,
        staleness_threshold=args_parsed.staleness_threshold,
        deduplicate_overlap=args_parsed.deduplicate_overlap,
        dedup_global_distance=args_parsed.dedup_global_distance
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
