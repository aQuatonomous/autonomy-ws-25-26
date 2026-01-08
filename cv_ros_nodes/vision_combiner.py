"""
Multi-Camera Detection Combiner ROS2 Node

Subscribes to: /camera0/detection_info, /camera1/detection_info, /camera2/detection_info
Publishes to: /combined/detection_info (String with all detections from all cameras)

This node combines detections from all 3 cameras into a single detection list.
Each detection includes its source camera_id for tracking.

OPERATION MODE:
- Latest Value Approach (default): Combines the most recent detection from each camera,
  regardless of when they were captured. This is fast and simple.
  
- Staleness Filtering: Automatically excludes detections from cameras that haven't
  updated recently. If a camera's latest detection is older than the staleness threshold,
  it's excluded from the output (indicating the camera may have failed or stalled).

Usage:
    # Default: Latest value with 1.0s staleness threshold
    python3 vision_combiner.py
    
    # Custom staleness threshold (0.5s - more strict)
    python3 vision_combiner.py --staleness_threshold 0.5
    
    # Very lenient (2.0s - allows slower cameras)
    python3 vision_combiner.py --staleness_threshold 2.0
    
    # Optional: Timestamp-based sync mode (more complex)
    python3 vision_combiner.py --use_timestamp_sync --sync_window 0.05
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict, List, Optional
import time


class DetectionCombiner(Node):
    
    def __init__(self, apply_nms: bool = False, iou_threshold: float = 0.5, 
                 sync_window: float = 0.05, use_timestamp_sync: bool = False,
                 staleness_threshold: float = 1.0):
        super().__init__('detection_combiner')
        
        self.apply_nms = apply_nms
        self.iou_threshold = iou_threshold
        self.use_timestamp_sync = use_timestamp_sync
        self.sync_window = sync_window  # Time window for timestamp matching (seconds)
        self.staleness_threshold = staleness_threshold  # Max age for detections (seconds)
        
        # Store latest detections from each camera
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
        
        # For timestamp-based synchronization: store detection history
        # Format: {timestamp: {camera_id: detection_data}}
        self.detection_history = []
        self.max_history_size = 100  # Keep last 100 detections per camera
        
        # Subscriptions for each camera's detection info
        self.subscriptions = []
        for camera_id in [0, 1, 2]:
            sub = self.create_subscription(
                String,
                f'/camera{camera_id}/detection_info',
                lambda msg, cid=camera_id: self.detection_callback(msg, cid),
                10
            )
            self.subscriptions.append(sub)
            self.get_logger().info(f'Subscribed to: /camera{camera_id}/detection_info')
        
        # Publisher for combined detections
        self.combined_pub = self.create_publisher(
            String,
            '/combined/detection_info',
            10
        )
        
        self.get_logger().info('Detection combiner node initialized')
        self.get_logger().info('Publishing to: /combined/detection_info')
        
        # Timer to periodically publish combined detections
        # This ensures we publish even if one camera is slow
        self.timer = self.create_timer(0.033, self.publish_combined)  # ~30 Hz
    
    def detection_callback(self, msg: String, camera_id: int):
        """Callback for individual camera detection info"""
        try:
            detection_data = json.loads(msg.data)
            detection_timestamp = detection_data.get('timestamp', time.time())
            
            # Store latest detection (for non-sync mode)
            self.detections[camera_id] = detection_data
            self.last_update_times[camera_id] = time.time()
            
            # If using timestamp sync, try to match with other cameras
            if self.use_timestamp_sync:
                self._try_synchronize_and_publish(detection_timestamp, camera_id, detection_data)
            else:
                # Old behavior: immediately publish when new data arrives
                self.publish_combined()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detection info from camera{camera_id}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing detection from camera{camera_id}: {e}')
    
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
    
    def calculate_iou(self, box1: List[float], box2: List[float]) -> float:
        """Calculate Intersection over Union (IoU) between two bounding boxes"""
        # Box format: [x1, y1, x2, y2]
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        if union == 0:
            return 0.0
        
        return intersection / union
    
    def apply_nms_across_cameras(self, all_detections: List[Dict]) -> List[Dict]:
        """Apply Non-Maximum Suppression across detections from all cameras"""
        if not all_detections:
            return []
        
        # Sort by confidence (highest first)
        sorted_detections = sorted(all_detections, key=lambda x: x['score'], reverse=True)
        
        kept = []
        while sorted_detections:
            # Keep the highest confidence detection
            current = sorted_detections.pop(0)
            kept.append(current)
            
            # Remove overlapping detections
            remaining = []
            for det in sorted_detections:
                iou = self.calculate_iou(current['bbox'], det['bbox'])
                if iou < self.iou_threshold:
                    remaining.append(det)
            sorted_detections = remaining
        
        return kept
    
    def _publish_synchronized(self, matched_detections: dict, matched_timestamps: dict):
        """Publish synchronized detections from matched cameras"""
        all_detections = []
        camera_stats = {}
        current_time = time.time()
        
        # Process matched detections
        for camera_id, detection_data in matched_detections.items():
            detections = detection_data.get('detections', [])
            for det in detections:
                det_with_camera = det.copy()
                det_with_camera['camera_id'] = camera_id
                all_detections.append(det_with_camera)
            
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('fps', 0.0),
                'timestamp': matched_timestamps[camera_id]
            }
        
        # Add stats for unmatched cameras
        for camera_id in [0, 1, 2]:
            if camera_id not in matched_detections:
                time_since_update = current_time - self.last_update_times[camera_id]
                if self.detections[camera_id] is None:
                    camera_stats[camera_id] = {'status': 'no_data', 'num_detections': 0}
                elif time_since_update > self.detection_timeout:
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
        
        # Apply NMS if requested
        if self.apply_nms and all_detections:
            all_detections = self.apply_nms_across_cameras(all_detections)
        
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
            'synchronized': True
        }
        
        # Publish combined detection info
        msg = String()
        msg.data = json.dumps(combined_info)
        self.combined_pub.publish(msg)
    
    def publish_combined(self):
        """
        Combine detections from all cameras and publish.
        Uses latest value approach with staleness filtering.
        Only includes detections from cameras that have updated recently.
        """
        current_time = time.time()
        all_detections = []
        camera_stats = {}
        active_cameras = 0
        stale_cameras = 0
        no_data_cameras = 0
        
        # Collect detections from all cameras
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
            
            # Camera is active and fresh - include its detections
            active_cameras += 1
            detections = detection_data.get('detections', [])
            for det in detections:
                det_with_camera = det.copy()
                det_with_camera['camera_id'] = camera_id
                all_detections.append(det_with_camera)
            
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('fps', 0.0),
                'timestamp': detection_data.get('timestamp', 0.0),
                'time_since_update': round(time_since_update, 3)
            }
        
        # Apply NMS if requested
        if self.apply_nms and all_detections:
            all_detections = self.apply_nms_across_cameras(all_detections)
        
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
            'synchronized': self.use_timestamp_sync if self.use_timestamp_sync else None
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
        
        if self._log_counter % 90 == 0:  # Log every ~3 seconds at 30Hz
            if stale_cameras > 0 or no_data_cameras > 0:
                self.get_logger().warn(
                    f'Camera health: {active_cameras} active, {stale_cameras} stale, '
                    f'{no_data_cameras} no data | Total detections: {len(all_detections)}'
                )
            else:
                self.get_logger().info(
                    f'Combined detections: {len(all_detections)} total | '
                    f'Active cameras: {active_cameras}/3 | '
                    f'Camera0: {camera_stats[0]["num_detections"]}, '
                    f'Camera1: {camera_stats[1]["num_detections"]}, '
                    f'Camera2: {camera_stats[2]["num_detections"]}'
                )
        
        # Log periodically
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 90 == 0:  # Log every ~3 seconds at 30Hz
            active_cameras = sum(1 for stat in camera_stats.values() if stat['status'] == 'active')
            self.get_logger().info(
                f'Combined detections: {len(all_detections)} total | '
                f'Active cameras: {active_cameras}/3 | '
                f'Camera0: {camera_stats[0]["num_detections"]}, '
                f'Camera1: {camera_stats[1]["num_detections"]}, '
                f'Camera2: {camera_stats[2]["num_detections"]}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser(description='ROS2 Multi-Camera Detection Combiner')
    parser.add_argument('--apply_nms', action='store_true',
                       help='Apply Non-Maximum Suppression across cameras to remove duplicates')
    parser.add_argument('--iou_threshold', type=float, default=0.5,
                       help='IoU threshold for NMS (0.0-1.0)')
    parser.add_argument('--staleness_threshold', type=float, default=1.0,
                       help='Maximum age (seconds) for detections before excluding camera (default: 1.0s)')
    parser.add_argument('--sync_window', type=float, default=0.05,
                       help='Time window (seconds) for timestamp-based synchronization (default: 0.05 = 50ms)')
    parser.add_argument('--use_timestamp_sync', action='store_true',
                       help='Enable timestamp-based synchronization (default: latest value approach)')
    args_parsed = parser.parse_args(args)
    
    node = DetectionCombiner(
        apply_nms=args_parsed.apply_nms,
        iou_threshold=args_parsed.iou_threshold,
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
