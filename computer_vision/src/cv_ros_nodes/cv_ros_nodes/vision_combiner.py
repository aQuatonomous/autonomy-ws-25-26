"""
Multi-Camera Detection Combiner ROS2 Node

Subscribes to: /camera0/detection_info, /camera1/detection_info, /camera2/detection_info;
  /camera0/task4_detections, /camera1/task4_detections, /camera2/task4_detections
Publishes to: /combined/detection_info (String with all detections; bbox in global frame 3×frame_width×frame_height, e.g. 5760×1200; global_frame from detection_info)

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

OVERLAP DEDUPLICATION:
For side-by-side cameras with 15° overlap, detections in overlap zones can be deduplicated.
- Camera 0 overlaps with Camera 1: right edge of Camera 0, left edge of Camera 1
- Camera 1 overlaps with Camera 2: right edge of Camera 1, left edge of Camera 2
- Overlap zones are defined as a percentage of frame width (configurable, default ~15%)
- Matching criteria: same class_id, similar vertical position (y-coordinate), both in overlap zones
- Keeps detection with higher confidence score

Usage:
    # Default: Combines all 3 cameras, 1.0s staleness threshold, NO overlap deduplication
    python3 vision_combiner.py
    
    # Enable overlap deduplication (recommended for side-by-side cameras)
    python3 vision_combiner.py --deduplicate_overlap --overlap_zone_width 0.15 --overlap_y_tolerance 0.1
    
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
FOV_PER_CAMERA_DEG = 85
OVERLAP_DEG = 15
EFFECTIVE_ANGLE_SPAN_DEG = 225  # 3*85 - 2*15
EFFECTIVE_ANGLE_LEFT_DEG = -112.5
EFFECTIVE_ANGLE_RIGHT_DEG = 112.5


class DetectionCombiner(Node):
    
    def __init__(self, sync_window: float = 0.05, use_timestamp_sync: bool = False,
                 staleness_threshold: float = 1.0, deduplicate_overlap: bool = False,
                 overlap_zone_width: float = 0.15, overlap_y_tolerance: float = 0.1):
        super().__init__('detection_combiner')
        
        self.use_timestamp_sync = use_timestamp_sync
        self.sync_window = sync_window  # Time window for timestamp matching (seconds)
        self.staleness_threshold = staleness_threshold  # Max age for detections (seconds)
        self.deduplicate_overlap = deduplicate_overlap  # Enable overlap zone deduplication
        self.overlap_zone_width = overlap_zone_width  # Fraction of frame width for overlap zone (0.15 = 15%)
        self.overlap_y_tolerance = overlap_y_tolerance  # Fraction of frame height for y-coordinate matching (0.1 = 10%)
        
        # Per-camera frame dimensions (learned from detection_info; fallback until first inference)
        self.frame_width = 640
        self.frame_height = 480
        # Global frame: 3 cameras side-by-side (0=left, 1=center, 2=right)
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
        Returns effective_x, effective_y (pixels), angle_deg (0=center, negative=left, positive=right), angle_rad.
        """
        if len(bbox) != 4:
            return {
                'effective_x': 0.0,
                'effective_y': 0.0,
                'angle_deg': 0.0,
                'angle_rad': 0.0,
            }
        raw_width = 3 * self.frame_width
        effective_width = round(raw_width * EFFECTIVE_ANGLE_SPAN_DEG / 255.0)
        x_center = (float(bbox[0]) + float(bbox[2])) / 2.0
        y_center = (float(bbox[1]) + float(bbox[3])) / 2.0
        x_raw = camera_id * self.frame_width + x_center
        effective_x = x_raw * effective_width / raw_width
        effective_y = y_center
        angle_deg = EFFECTIVE_ANGLE_LEFT_DEG + (x_raw / raw_width) * EFFECTIVE_ANGLE_SPAN_DEG
        angle_rad = math.radians(angle_deg)
        return {
            'effective_x': effective_x,
            'effective_y': effective_y,
            'angle_deg': angle_deg,
            'angle_rad': angle_rad,
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
    
    def _inference_bbox_to_640x480(self, bbox: List[float]) -> List[float]:
        """
        Convert bounding box from inference space (640x640) to preprocessing space (640x480).
        
        The inference model resizes images to 640x640 internally, but preprocessing outputs
        640x480. This method scales the y-coordinates accordingly.
        
        Args:
            bbox: [x1, y1, x2, y2] bounding box in 640x640 space
        
        Returns:
            [x1, y1, x2, y2] bounding box in 640x480 space
        """
        if len(bbox) != 4:
            return bbox
        
        x1, y1, x2, y2 = bbox
        # X coordinates stay the same (both are 640 wide)
        # Y coordinates scale by 480/640 = 0.75
        scale_y = 480.0 / 640.0
        y1_scaled = y1 * scale_y
        y2_scaled = y2 * scale_y
        
        return [x1, y1_scaled, x2, y2_scaled]
    
    def _is_in_overlap_zone(self, bbox: List[float], camera_id: int, is_left: bool) -> bool:
        """
        Check if a detection is in the overlap zone of a camera.
        
        Args:
            bbox: [x1, y1, x2, y2] bounding box coordinates
            camera_id: Camera ID (0, 1, or 2)
            is_left: True if checking left edge, False if checking right edge
        
        Returns:
            True if detection is in the overlap zone
        """
        x1, y1, x2, y2 = bbox
        bbox_center_x = (x1 + x2) / 2.0
        
        if is_left:
            # Left edge overlap zone (for Camera 1 and Camera 2)
            overlap_threshold = self.frame_width * self.overlap_zone_width
            return bbox_center_x < overlap_threshold
        else:
            # Right edge overlap zone (for Camera 0 and Camera 1)
            overlap_threshold = self.frame_width * (1.0 - self.overlap_zone_width)
            return bbox_center_x > overlap_threshold
    
    def _detections_match(self, det1: Dict, det2: Dict) -> bool:
        """
        Check if two detections from adjacent cameras match (same object in overlap zone).
        
        Matching criteria:
        1. Same class_id
        2. Similar vertical position (y-coordinate within tolerance)
        3. Both in overlap zones of their respective cameras
        
        Args:
            det1: First detection with camera_id and bbox
            det2: Second detection with camera_id and bbox
        
        Returns:
            True if detections likely represent the same object
        """
        # Must be same class
        if det1.get('class_id') != det2.get('class_id'):
            return False
        
        # Must be from adjacent cameras
        cam1_id = det1.get('camera_id')
        cam2_id = det2.get('camera_id')
        if abs(cam1_id - cam2_id) != 1:
            return False
        
        bbox1 = det1.get('bbox', [])
        bbox2 = det2.get('bbox', [])
        if len(bbox1) != 4 or len(bbox2) != 4:
            return False
        
        # Check if both are in overlap zones
        # Camera 0 overlaps with Camera 1: right edge of Camera 0, left edge of Camera 1
        # Camera 1 overlaps with Camera 2: right edge of Camera 1, left edge of Camera 2
        if cam1_id == 0 and cam2_id == 1:
            # Camera 0 (right edge) and Camera 1 (left edge)
            in_overlap1 = self._is_in_overlap_zone(bbox1, cam1_id, is_left=False)
            in_overlap2 = self._is_in_overlap_zone(bbox2, cam2_id, is_left=True)
        elif cam1_id == 1 and cam2_id == 0:
            # Camera 1 (left edge) and Camera 0 (right edge)
            in_overlap1 = self._is_in_overlap_zone(bbox1, cam1_id, is_left=True)
            in_overlap2 = self._is_in_overlap_zone(bbox2, cam2_id, is_left=False)
        elif cam1_id == 1 and cam2_id == 2:
            # Camera 1 (right edge) and Camera 2 (left edge)
            in_overlap1 = self._is_in_overlap_zone(bbox1, cam1_id, is_left=False)
            in_overlap2 = self._is_in_overlap_zone(bbox2, cam2_id, is_left=True)
        elif cam1_id == 2 and cam2_id == 1:
            # Camera 2 (left edge) and Camera 1 (right edge)
            in_overlap1 = self._is_in_overlap_zone(bbox1, cam1_id, is_left=True)
            in_overlap2 = self._is_in_overlap_zone(bbox2, cam2_id, is_left=False)
        else:
            return False
        
        if not (in_overlap1 and in_overlap2):
            return False
        
        # Check vertical position similarity (y-coordinate)
        y1_center = (bbox1[1] + bbox1[3]) / 2.0
        y2_center = (bbox2[1] + bbox2[3]) / 2.0
        y_diff = abs(y1_center - y2_center)
        y_tolerance = self.frame_height * self.overlap_y_tolerance
        
        return y_diff <= y_tolerance
    
    def _deduplicate_overlap_zones(self, all_detections: List[Dict]) -> List[Dict]:
        """
        Remove duplicate detections in overlap zones between adjacent cameras.
        
        For side-by-side cameras with 15° overlap:
        - Camera 0 right edge overlaps with Camera 1 left edge
        - Camera 1 right edge overlaps with Camera 2 left edge
        
        When two detections match (same class, similar position, both in overlap zones),
        keeps the one with higher confidence score.
        
        Args:
            all_detections: List of all detections from all cameras
        
        Returns:
            Filtered list with duplicates removed
        """
        if not all_detections:
            return []
        
        # Sort by confidence (highest first) so we keep the best detections
        sorted_detections = sorted(all_detections, key=lambda x: x.get('score', 0.0), reverse=True)
        
        kept = []
        removed_count = 0
        
        for current_det in sorted_detections:
            is_duplicate = False
            
            # Check against all already-kept detections
            for kept_det in kept:
                if self._detections_match(current_det, kept_det):
                    # Found a match - this is a duplicate
                    is_duplicate = True
                    removed_count += 1
                    break
            
            if not is_duplicate:
                kept.append(current_det)
        
        if removed_count > 0:
            self.get_logger().debug(f'Overlap deduplication: removed {removed_count} duplicate detection(s)')
        
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
                # Convert bbox from inference space (640x640) to preprocessing space (640x480)
                bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
                det_with_camera['bbox'] = self._inference_bbox_to_640x480(bbox)
                all_detections.append(det_with_camera)
            
            # Add stats for matched cameras
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('fps', 0.0),
                'timestamp': matched_timestamps[camera_id]
            }
        
        # Apply overlap deduplication if enabled
        if self.deduplicate_overlap and len(all_detections) > 0:
            all_detections = self._deduplicate_overlap_zones(all_detections)

        # Effective global frame and per-detection effective coords + angle
        effective_width = int(round((3 * self.frame_width) * EFFECTIVE_ANGLE_SPAN_DEG / 255.0))
        effective_height = self.frame_height
        effective_global_frame = {
            'width': effective_width,
            'height': effective_height,
            'effective_angle_span_deg': EFFECTIVE_ANGLE_SPAN_DEG,
            'effective_angle_left_deg': EFFECTIVE_ANGLE_LEFT_DEG,
            'effective_angle_right_deg': EFFECTIVE_ANGLE_RIGHT_DEG,
        }
        for det in all_detections:
            eff = self._to_effective_global(det.get('camera_id', 0), det.get('bbox', []))
            det['effective_x'] = eff['effective_x']
            det['effective_y'] = eff['effective_y']
            det['angle_deg'] = eff['angle_deg']
            det['angle_rad'] = eff['angle_rad']
        
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
            
            # Camera is active and fresh - include ALL its detections (inference bbox 640x640 -> 640x480)
            # Camera is active and fresh - include ALL its detections (inference bbox 640x640 -> 640x480)
            active_cameras += 1
            detections = detection_data.get('detections', [])
            for det in detections:
                det_with_camera = det.copy()
                det_with_camera['camera_id'] = camera_id  # Tag each detection with source camera
                # Convert bbox from inference space (640x640) to preprocessing space (640x480)
                bbox = det.get('bbox') or [det.get('x1', 0), det.get('y1', 0), det.get('x2', 0), det.get('y2', 0)]
                det_with_camera['bbox'] = self._inference_bbox_to_640x480(bbox)
                all_detections.append(det_with_camera)  # Add to combined list
            
            # Add stats for active camera
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('fps', 0.0),
                'timestamp': detection_data.get('timestamp', 0.0),
                'time_since_update': round(time_since_update, 3)
            }
        
        # Apply overlap deduplication if enabled
        if self.deduplicate_overlap and len(all_detections) > 0:
            all_detections = self._deduplicate_overlap_zones(all_detections)

        # Effective global frame (225° span, ~5082 px for 1920 per camera)
        effective_width = int(round((3 * self.frame_width) * EFFECTIVE_ANGLE_SPAN_DEG / 255.0))
        effective_height = self.frame_height
        effective_global_frame = {
            'width': effective_width,
            'height': effective_height,
            'effective_angle_span_deg': EFFECTIVE_ANGLE_SPAN_DEG,
            'effective_angle_left_deg': EFFECTIVE_ANGLE_LEFT_DEG,
            'effective_angle_right_deg': EFFECTIVE_ANGLE_RIGHT_DEG,
        }
        for det in all_detections:
            eff = self._to_effective_global(det.get('camera_id', 0), det.get('bbox', []))
            det['effective_x'] = eff['effective_x']
            det['effective_y'] = eff['effective_y']
            det['angle_deg'] = eff['angle_deg']
            det['angle_rad'] = eff['angle_rad']
        
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
                       help='Enable overlap zone deduplication for side-by-side cameras (default: disabled)')
    parser.add_argument('--overlap_zone_width', type=float, default=0.15,
                       help='Fraction of frame width for overlap zone (default: 0.15 = 15%%)')
    parser.add_argument('--overlap_y_tolerance', type=float, default=0.1,
                       help='Fraction of frame height for y-coordinate matching tolerance (default: 0.1 = 10%%)')
    args_parsed, _ = parser.parse_known_args(args=args)
    
    node = DetectionCombiner(
        sync_window=args_parsed.sync_window,
        use_timestamp_sync=args_parsed.use_timestamp_sync,
        staleness_threshold=args_parsed.staleness_threshold,
        deduplicate_overlap=args_parsed.deduplicate_overlap,
        overlap_zone_width=args_parsed.overlap_zone_width,
        overlap_y_tolerance=args_parsed.overlap_y_tolerance
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
