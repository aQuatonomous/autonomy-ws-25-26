"""
Multi-Camera Detection Combiner ROS2 Node

Subscribes to: /camera0/detection_info, /camera1/detection_info, /camera2/detection_info
Publishes to: /combined/detection_info (String with all detections from all cameras)

This node combines detections from all 3 cameras into a single detection list.
Each detection includes its source camera_id for tracking.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from typing import Dict, List, Optional
import time


class DetectionCombiner(Node):
    
    def __init__(self, apply_nms: bool = False, iou_threshold: float = 0.5):
        super().__init__('detection_combiner')
        
        self.apply_nms = apply_nms
        self.iou_threshold = iou_threshold
        
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
        
        # Timeout for stale detections (seconds)
        self.detection_timeout = 1.0
        
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
            self.detections[camera_id] = detection_data
            self.last_update_times[camera_id] = time.time()
            
            # Immediately publish combined detections when new data arrives
            self.publish_combined()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detection info from camera{camera_id}: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing detection from camera{camera_id}: {e}')
    
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
    
    def publish_combined(self):
        """Combine detections from all cameras and publish"""
        current_time = time.time()
        all_detections = []
        camera_stats = {}
        
        # Collect detections from all cameras
        for camera_id in [0, 1, 2]:
            detection_data = self.detections[camera_id]
            
            # Check if detection is stale
            time_since_update = current_time - self.last_update_times[camera_id]
            is_stale = time_since_update > self.detection_timeout
            
            if detection_data is None:
                camera_stats[camera_id] = {
                    'status': 'no_data',
                    'num_detections': 0
                }
                continue
            
            if is_stale:
                camera_stats[camera_id] = {
                    'status': 'stale',
                    'num_detections': 0,
                    'time_since_update': time_since_update
                }
                continue
            
            # Extract detections and add camera_id to each
            detections = detection_data.get('detections', [])
            for det in detections:
                det_with_camera = det.copy()
                det_with_camera['camera_id'] = camera_id
                all_detections.append(det_with_camera)
            
            camera_stats[camera_id] = {
                'status': 'active',
                'num_detections': len(detections),
                'fps': detection_data.get('fps', 0.0),
                'timestamp': detection_data.get('timestamp', 0.0)
            }
        
        # Apply NMS if requested
        if self.apply_nms and all_detections:
            all_detections = self.apply_nms_across_cameras(all_detections)
        
        # Create combined detection info
        combined_info = {
            'timestamp': current_time,
            'num_cameras': 3,
            'total_detections': len(all_detections),
            'camera_stats': camera_stats,
            'detections': all_detections
        }
        
        # Publish combined detection info
        msg = String()
        msg.data = json.dumps(combined_info)
        self.combined_pub.publish(msg)
        
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
    args_parsed = parser.parse_args(args)
    
    node = DetectionCombiner(
        apply_nms=args_parsed.apply_nms,
        iou_threshold=args_parsed.iou_threshold
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
