#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class FOVTester(Node):
    def __init__(self):
        super().__init__('fov_tester')
        self.sub = self.create_subscription(
            PointCloud2, '/points_filtered', self.callback, 10)
        self.count = 0
        
    def callback(self, msg):
        self.count += 1
        if self.count % 10 != 0:  # Only check every 10th message
            return
            
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if len(pts) == 0:
            return
            
        x_vals = [p[0] for p in pts]
        y_vals = [p[1] for p in pts]
        z_vals = [p[2] for p in pts]
        
        # Calculate angles from +X
        angles = []
        for i in range(len(x_vals)):
            perp_dist = np.sqrt(y_vals[i]**2 + z_vals[i]**2)
            angle_rad = np.arctan2(perp_dist, x_vals[i])
            angle_deg = np.degrees(angle_rad)
            angles.append(angle_deg)
        
        negative_x = [x for x in x_vals if x < 0]
        
        self.get_logger().info(
            f'Points: {len(pts)} | '
            f'X range: [{min(x_vals):.2f}, {max(x_vals):.2f}] | '
            f'Angle range: [{min(angles):.1f}°, {max(angles):.1f}°] | '
            f'Negative X count: {len(negative_x)}'
        )
        
        if len(negative_x) > 0:
            self.get_logger().warn(f'Found {len(negative_x)} points with negative X (behind)!')

def main():
    rclpy.init()
    node = FOVTester()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main()
