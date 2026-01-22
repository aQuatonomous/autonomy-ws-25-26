#!/usr/bin/env python3
"""
Buoy Tracker Node

Maintains persistent tracking of detected buoys with stable IDs across frames.
Performs data association to match new detections with existing tracked buoys.
Supports data fusion with vision system for color information.

Subscribes to:
    /buoy_detections (pointcloud_filters/BuoyDetectionArray): Raw detections from DBSCAN
    /buoy_colors (optional, future): Color information from vision system

Publishes:
    /tracked_buoys (pointcloud_filters/TrackedBuoyArray): Buoys with persistent IDs
"""

import math
from typing import Dict, Set, Tuple, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from pointcloud_filters.msg import BuoyDetectionArray, BuoyDetection
from pointcloud_filters.msg import TrackedBuoyArray, TrackedBuoy


class BuoyTracker(Node):
    """
    Tracks buoys across frames with persistent IDs using nearest-neighbor data association.
    
    Maintains a hashmap of tracked buoys, updating positions when re-detected and
    removing buoys that haven't been seen for several frames.
    """
    
    def __init__(self):
        super().__init__('buoy_tracker')
        
        # Parameters
        self.declare_parameter('association_threshold', 0.76)  # 2.5 feet in meters
        self.declare_parameter('max_consecutive_misses', 10)  # Frames before removal
        self.declare_parameter('position_alpha', 0.7)  # Exponential smoothing (0=old, 1=new)
        self.declare_parameter('min_observations_for_publish', 3)  # Require N detections before publishing
        
        self.association_threshold = self.get_parameter('association_threshold').value
        self.max_consecutive_misses = self.get_parameter('max_consecutive_misses').value
        self.position_alpha = self.get_parameter('position_alpha').value
        self.min_observations = self.get_parameter('min_observations_for_publish').value
        
        # Tracked buoys storage: {id: buoy_data_dict}
        self.tracked_buoys: Dict[int, dict] = {}
        self.next_id = 0  # Counter for assigning new IDs
        
        # QoS profile
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Subscriber to raw detections
        self.detection_sub = self.create_subscription(
            BuoyDetectionArray,
            '/buoy_detections',
            self.detection_callback,
            qos
        )
        
        # Publisher for tracked buoys
        self.tracked_pub = self.create_publisher(
            TrackedBuoyArray,
            '/tracked_buoys',
            qos
        )
        
        self.get_logger().info('Buoy tracker started')
        self.get_logger().info(f'Association threshold: {self.association_threshold:.2f}m, '
                              f'Max misses: {self.max_consecutive_misses}, '
                              f'Position smoothing alpha: {self.position_alpha}')
    
    def detection_callback(self, msg: BuoyDetectionArray):
        """
        Process new detections: associate with tracked buoys, update existing, add new, remove lost.
        """
        # Convert detections to list with Cartesian coordinates
        new_detections = []
        for det in msg.detections:
            x = det.range * math.cos(det.bearing)
            y = det.range * math.sin(det.bearing)
            new_detections.append({
                'range': det.range,
                'bearing': det.bearing,
                'x': x,
                'y': y,
                'z_mean': det.z_mean,
                'confidence': det.confidence,
                'num_points': det.num_points,
                'detection': det  # Keep original
            })
        
        # Data association
        matches, unmatched_detections, unmatched_buoys = self.associate_detections(new_detections)
        
        current_time = Time.from_msg(msg.header.stamp)
        
        # Update matched buoys
        for buoy_id, det_idx in matches.items():
            self.update_buoy(buoy_id, new_detections[det_idx], current_time)
        
        # Add new buoys from unmatched detections
        for det_idx in unmatched_detections:
            self.add_new_buoy(new_detections[det_idx], current_time)
        
        # Increment miss counter for unmatched buoys
        for buoy_id in unmatched_buoys:
            self.tracked_buoys[buoy_id]['consecutive_misses'] += 1
        
        # Remove buoys with too many consecutive misses
        to_remove = [bid for bid, buoy in self.tracked_buoys.items() 
                     if buoy['consecutive_misses'] > self.max_consecutive_misses]
        for bid in to_remove:
            self.get_logger().info(f'Removing buoy {bid} (not seen for {self.max_consecutive_misses} frames)')
            del self.tracked_buoys[bid]
        
        # Publish tracked buoys
        self.publish_tracked_buoys(msg.header)
        
        # Log summary
        self.get_logger().info(
            f'Tracking: {len(self.tracked_buoys)} buoys | '
            f'Matched: {len(matches)} | New: {len(unmatched_detections)} | Lost: {len(to_remove)}'
        )
    
    def associate_detections(self, new_detections: List[dict]) -> Tuple[Dict[int, int], Set[int], Set[int]]:
        """
        Associate new detections with tracked buoys using nearest-neighbor matching.
        
        Returns:
            matches: {buoy_id: detection_index}
            unmatched_detections: set of detection indices
            unmatched_buoys: set of buoy IDs
        """
        matches = {}
        unmatched_detections = set(range(len(new_detections)))
        unmatched_buoys = set(self.tracked_buoys.keys())
        
        # Build distance matrix and find best matches
        # Simple greedy nearest-neighbor (could upgrade to Hungarian algorithm)
        for buoy_id, buoy in self.tracked_buoys.items():
            best_dist = float('inf')
            best_idx = None
            
            for det_idx in list(unmatched_detections):
                detection = new_detections[det_idx]
                dist = self.compute_distance_3d(buoy, detection)
                
                if dist < best_dist and dist < self.association_threshold:
                    best_dist = dist
                    best_idx = det_idx
            
            if best_idx is not None:
                matches[buoy_id] = best_idx
                unmatched_detections.remove(best_idx)
                unmatched_buoys.remove(buoy_id)
        
        return matches, unmatched_detections, unmatched_buoys
    
    def compute_distance_3d(self, buoy: dict, detection: dict) -> float:
        """
        Compute 3D Euclidean distance between tracked buoy and detection.
        Uses (x, y, range) to distinguish buoys at same bearing but different distances.
        """
        dx = buoy['x'] - detection['x']
        dy = buoy['y'] - detection['y']
        dr = buoy['range'] - detection['range']
        
        return math.sqrt(dx*dx + dy*dy + dr*dr)
    
    def update_buoy(self, buoy_id: int, detection: dict, current_time: Time):
        """
        Update existing buoy with new detection using exponential smoothing for position.
        """
        buoy = self.tracked_buoys[buoy_id]
        alpha = self.position_alpha
        
        # Smooth position (exponential moving average)
        buoy['x'] = alpha * detection['x'] + (1 - alpha) * buoy['x']
        buoy['y'] = alpha * detection['y'] + (1 - alpha) * buoy['y']
        buoy['z_mean'] = alpha * detection['z_mean'] + (1 - alpha) * buoy['z_mean']
        
        # Recompute polar coordinates from smoothed Cartesian
        buoy['range'] = math.sqrt(buoy['x']**2 + buoy['y']**2)
        buoy['bearing'] = math.atan2(buoy['y'], buoy['x'])
        
        # Update confidence (take max or average)
        buoy['confidence'] = max(buoy['confidence'], detection['confidence'])
        
        # Update timestamps and counters
        buoy['last_seen'] = current_time
        buoy['observation_count'] += 1
        buoy['consecutive_misses'] = 0  # Reset miss counter
    
    def add_new_buoy(self, detection: dict, current_time: Time):
        """
        Add a new buoy to the tracked set with a fresh unique ID.
        """
        buoy_id = self.next_id
        self.next_id += 1
        
        self.tracked_buoys[buoy_id] = {
            'id': buoy_id,
            'range': detection['range'],
            'bearing': detection['bearing'],
            'x': detection['x'],
            'y': detection['y'],
            'z_mean': detection['z_mean'],
            'color': 'unknown',  # Will be set by vision system
            'confidence': detection['confidence'],
            'observation_count': 1,
            'first_seen': current_time,
            'last_seen': current_time,
            'consecutive_misses': 0
        }
        
        self.get_logger().info(
            f'New buoy {buoy_id} at range={detection["range"]:.2f}m, '
            f'bearing={math.degrees(detection["bearing"]):.1f}°'
        )
    
    def publish_tracked_buoys(self, header):
        """
        Publish tracked buoys that meet minimum observation threshold.
        """
        msg = TrackedBuoyArray()
        msg.header = header
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for buoy_id, buoy in self.tracked_buoys.items():
            # Only publish buoys with enough observations (reduces false positives)
            if buoy['observation_count'] >= self.min_observations:
                tracked_msg = TrackedBuoy()
                tracked_msg.id = buoy['id']
                tracked_msg.range = buoy['range']
                tracked_msg.bearing = buoy['bearing']
                tracked_msg.x = buoy['x']
                tracked_msg.y = buoy['y']
                tracked_msg.z_mean = buoy['z_mean']
                tracked_msg.color = buoy['color']
                tracked_msg.confidence = buoy['confidence']
                tracked_msg.observation_count = buoy['observation_count']
                tracked_msg.first_seen = buoy['first_seen'].to_msg()
                tracked_msg.last_seen = buoy['last_seen'].to_msg()
                
                msg.buoys.append(tracked_msg)
        
        self.tracked_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
