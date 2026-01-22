#!/usr/bin/env python3
"""
Tracked Buoy Visualizer Node

Visualizes tracked buoys with stable IDs and color coding in RViz.
Shows persistent buoy entities with their assigned colors from vision system.

Subscribes to:
    /tracked_buoys (pointcloud_filters/TrackedBuoyArray): Tracked buoys with IDs

Publishes:
    /tracked_buoy_markers (visualization_msgs/MarkerArray): Visualization markers
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from pointcloud_filters.msg import TrackedBuoyArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math


class TrackedBuoyVisualizer(Node):
    """
    Visualizes tracked buoys with stable IDs and color-coded markers.
    
    Marker colors:
    - Red buoys: Red cylinder
    - Green buoys: Green cylinder
    - Yellow buoys: Yellow cylinder
    - Unknown: White cylinder
    
    Marker IDs are persistent across frames (same buoy = same ID).
    """
    
    def __init__(self):
        super().__init__('tracked_buoy_visualizer')
        
        # Parameters
        self.declare_parameter('marker_height', 0.3)
        self.declare_parameter('marker_radius', 0.1)
        self.declare_parameter('marker_lifetime', 2.0)  # Longer lifetime for tracked buoys
        self.declare_parameter('show_text_labels', True)
        
        self.marker_height = self.get_parameter('marker_height').value
        self.marker_radius = self.get_parameter('marker_radius').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.show_text = self.get_parameter('show_text_labels').value
        
        # Color mapping
        self.color_map = {
            'red': (1.0, 0.0, 0.0),
            'green': (0.0, 1.0, 0.0),
            'yellow': (1.0, 1.0, 0.0),
            'unknown': (1.0, 1.0, 1.0),  # White
        }
        
        # Track which IDs we've seen to manage marker deletion
        self.active_ids = set()
        
        # QoS profile
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Subscriber to tracked buoys
        self.sub = self.create_subscription(
            TrackedBuoyArray,
            '/tracked_buoys',
            self.tracked_callback,
            qos
        )
        
        # Publisher for markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/tracked_buoy_markers',
            qos
        )
        
        self.get_logger().info('Tracked buoy visualizer started')
        self.get_logger().info(f'Marker: height={self.marker_height}m, radius={self.marker_radius}m, '
                              f'lifetime={self.marker_lifetime}s, text_labels={self.show_text}')
    
    def tracked_callback(self, msg: TrackedBuoyArray):
        """
        Create markers for each tracked buoy with stable IDs and color coding.
        """
        marker_array = MarkerArray()
        current_ids = set()
        
        for buoy in msg.buoys:
            current_ids.add(buoy.id)
            
            # Cylinder marker for buoy body
            marker = Marker()
            marker.header = msg.header
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = buoy.id  # Use persistent buoy ID
            marker.ns = "tracked_buoys"
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = buoy.x
            marker.pose.position.y = buoy.y
            marker.pose.position.z = buoy.z_mean
            marker.pose.orientation.w = 1.0
            
            # Scale
            marker.scale.x = self.marker_radius * 2.0
            marker.scale.y = self.marker_radius * 2.0
            marker.scale.z = self.marker_height
            
            # Color based on buoy color with confidence as intensity
            # Higher confidence = darker/more saturated color
            rgb = self.color_map.get(buoy.color.lower(), self.color_map['unknown'])
            confidence_intensity = min(1.0, max(0.3, buoy.confidence))  # Clamp to [0.3, 1.0]
            
            # Scale RGB values by confidence (higher confidence = darker/fuller color)
            marker.color.r = rgb[0] * confidence_intensity
            marker.color.g = rgb[1] * confidence_intensity
            marker.color.b = rgb[2] * confidence_intensity
            marker.color.a = 0.9  # Keep mostly opaque
            
            # Lifetime
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime % 1.0) * 1e9)
            
            marker_array.markers.append(marker)
            
            # Text label showing ID and observation count
            if self.show_text:
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.id = buoy.id + 10000  # Offset ID to avoid collision
                text_marker.ns = "buoy_labels"
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # Position above the cylinder
                text_marker.pose.position.x = buoy.x
                text_marker.pose.position.y = buoy.y
                text_marker.pose.position.z = buoy.z_mean + self.marker_height + 0.2
                text_marker.pose.orientation.w = 1.0
                
                # Text content
                text_marker.text = f"ID{buoy.id}\n({buoy.observation_count})"
                
                # Scale (text height)
                text_marker.scale.z = 0.15
                
                # Color (white text)
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                text_marker.lifetime = marker.lifetime
                
                marker_array.markers.append(text_marker)
        
        # Delete markers for buoys that disappeared
        deleted_ids = self.active_ids - current_ids
        for deleted_id in deleted_ids:
            # Delete cylinder
            delete_marker = Marker()
            delete_marker.header = msg.header
            delete_marker.id = deleted_id
            delete_marker.ns = "tracked_buoys"
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)
            
            # Delete text label
            if self.show_text:
                delete_text = Marker()
                delete_text.header = msg.header
                delete_text.id = deleted_id + 10000
                delete_text.ns = "buoy_labels"
                delete_text.action = Marker.DELETE
                marker_array.markers.append(delete_text)
        
        # Update active IDs
        self.active_ids = current_ids
        
        # Publish
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)
        
        self.get_logger().info(
            f'Visualizing {len(msg.buoys)} tracked buoys | '
            f'Active IDs: {sorted(current_ids)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackedBuoyVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
