#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import ros_numpy
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

    def callback(self, msg):
        # Convert PointCloud2 → NumPy array
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
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
