#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/unilidar/cloud',  # Topic name from your LiDAR
            self.callback,
            10)
        self.get_logger().info("Subscribed to /unilidar/cloud")
        self.subscription 
    def callback(self, msg):
        # Convert PointCloud2 to XYZ using read_points (handles mixed datatypes)
        iterable = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )
        # Build an array efficiently from iterator
        points = np.fromiter(
            (coord for p in iterable for coord in p),
            dtype=np.float32
        )
        if points.size == 0:
            self.get_logger().warn("Received empty point cloud")
            return
        points = points.reshape((-1, 3))
        self.get_logger().info(f"Received {points.shape[0]} points")

        # Example: Compute average distance
        distances = np.linalg.norm(points, axis=1)
        avg_dist = np.mean(distances)
        self.get_logger().info(f"Average distance: {avg_dist:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
