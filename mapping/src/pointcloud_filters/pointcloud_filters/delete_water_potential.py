"""
Delete Water Node - Removes water points from LiDAR point cloud

Subscribes to: /unilidar/cloud (sensor_msgs/PointCloud2): Raw LiDAR point cloud with water points
Publishes to: /points_no_water (sensor_msgs/PointCloud2): Filtered LiDAR point cloud without water points

Algorithm:
1. Define water plane from three calibration points.
2. Comuted signed distance of each point to the plane
3. Keep points above the plane (non-water points)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

LIDAR_HEIGHT = 0.5  # Meters, not actual value, just for reference
LIDAR_ANGLE = 15  # Degrees, not actual value, just for reference
LIDAR_ANGLE_RAD = math.radians(LIDAR_ANGLE) # Radians

class DeleteWaterNode(Node):
    def __init__(self):
        super().__init__('delete_water')

        # Parameter
        self.declare_parameter('input_topic', '/unilidar/cloud')
        self.declare_parameter('output_topic', '/points_no_water')

        self.declare_parameter('p1_x', 3.0)
        self.declare_parameter('p1_y', 0.0)
        self.declare_parameter('p1_z', -0.8)

        self.declare_parameter('p2_x', 5.0)
        self.declare_parameter('p2_y', -4.0)
        self.declare_parameter('p2_z', -1.34)

        self.declare_parameter('p3_x', 7.0)
        self.declare_parameter('p3_y', 2.0)
        self.declare_parameter('p3_z', -1.88)

        #Safety margin: points within this distance of the plane are also removed
        self.declare_parameter('safety_margin', 0.05) #5 cm margin

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.margin = self.get_parameter('safety_margin').value

        # Build water plane equation
        p1 = np.array([self.get_parameter('p1_x').value, self.get_parameter('p1_y').value, self.get_parameter('p1_z').value])
        p2 = np.array([self.get_parameter('p2_x').value, self.get_parameter('p2_y').value, self.get_parameter('p2_z').value])
        p3 = np.array([self.get_parameter('p3_x').value, self.get_parameter('p3_y').value, self.get_parameter('p3_z').value])

        # Compute plane equation Ax + By + Cz + D = 0
        self.a, self.b, self.c, self.d = self.compute_plane_equation(p1, p2, p3)

        # QoS matchin LiDAR driver'
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos
        )

        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos
        )

        self.get_logger().info(f'Delete Water Node started')
        self.get_logger().info(f'Water plane equation: {self.a}x + {self.b}y + {self.c}z + {self.d} = 0')
        self.get_logger().info(f'Normal vector: ({self.a}, {self.b}, {self.c})')
        self.get_logger().info(f'Margin: {self.margin} m')

    def compute_plane_equation(self, p1, p2, p3):
        #vectors
        v1 = p2 - p1
        v2 = p3 - p1

        normal = np.cross(v1, v2)

        normal_magnitude = np.linalg.norm(normal)

        if normal_magnitude == 0:
            self.get_logger().error('Calibration points are collinear, cannot define a plane.')
            raise ValueError('Calibration points are collinear')
        
        normal = normal / normal_magnitude

        # Ensure normal points upwards (positive Z)
        if(normal[2] < 0):
            normal = -normal
            self.get_logger().info('Inverted normal vector to point upwards')

        a, b, c = normal

        d = -(a * p1[0] + b * p1[1] + c * p1[2])

        return a, b, c, d
    
    #find distnace between point and plane
    def point_to_plane_distance(self, points):
        distances = (self.a * points[:, 0] + self.b * points[:, 1] + self.c * points[:, 2] + self.d)
        return distances


    # process and remove points from point cloud
    def cloud_callback(self, msg: PointCloud2):
        points = self.extract_points(msg)

        if len(points) == 0:
            self.get_logger().debug('Received empty point cloud')
            return
        
        # find distance to water plane
        distances = self.point_to_plane_distance(points)

        # Create mask for points above the plane + margin
        mask = distances > self.margin

        filtered_points = points[mask]

        removed_count = len(points) - len(filtered_points)
        removed_percentage = (removed_count / len(points)) * 100

        self.get_logger().info(f'Points: {len(points)} -> {len(filtered_points)}' f' | Removed: {removed_count}, {removed_percentage:.2f}%')

        self.publish_cloud(filtered_points, msg.header)

    # extract points from PointCloud2 message
    def extract_points(self, msg: PointCloud2):
        try:
            pts_iter = pc2.read_points(msg, field_names = ('x', 'y', 'z'), skip_nans=True)
        except Exception as e:
            self.get_logger().error(f'Error reading point cloud: {e}')
            return np.array([])

        values = []
        for p in pts_iter:
            if hasattr(p, 'dtype') and p.dtype.names:
                values.append((float(p['x']), float(p['y']), float(p['z'])))
            else:
                values.append((float(p[0]), float(p[1]), float(p[2])))

        if len(values) == 0:
            return np.array([])

        return np.array(values, dtype=np.float64)

    def publish_cloud(self, points, header):
        if len(points) == 0:
            out_msg = pc2.create_cloud_xyz32(header, [])
        else:
            points_list = points.astype(np.float32).tolist()
            out_msg = pc2.create_cloud_xyz32(header, points_list)

        self.pub.publish(out_msg)

    # visual water plane in RViz
    def publish_plane_marker(self):
        marker = Marker()
        marker.header.frame_id = 'unilidar_lidar'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "water_plane"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        plane_size = 20.0
        grid_divisions = 10

        x_min, x_max = 0.0, plane_size
        y_min, y_max = -plane_size / 2, plane_size / 2

        # generate grid of points on the plane, for each grid cell create two triangles
        step_x = (x_max - x_min) / grid_divisions
        step_y = (y_max - y_min) / grid_divisions

        for i in range(grid_divisions):
            for j in range(grid_divisions):
                x1 = x_min + i * step_x
                x2 = x_min + (i + 1) * step_x
                y1 = y_min + j * step_y
                y2 = y_min + (j + 1) * step_y

                #find x coordinates using plane equation
                if abs(self.c) < 1e-6:
                    self.get_logger().warn('Plane is vertical, cannot visualize')
                    return
                
                z1_1 = -(self.a * x1 + self.b * y1 + self.d) / self.c
                z1_2 = -(self.a * x2 + self.b * y1 + self.d) / self.c
                z2_1 = -(self.a * x2 + self.b * y2 + self.d) / self.c
                z2_2 = -(self.a * x1 + self.b * y2 + self.d) / self.c

                p1 = Point(x = x1, y = y1, z = z1_1)
                p2 = Point(x = x2, y = y1, z = z1_2)
                p3 = Point(x = x2, y = y2, z = z2_1)
                p4 = Point(x = x1, y = y2, z = z2_2)

                marker.points.append(p1)
                marker.points.append(p2)
                marker.points.append(p3)

                marker.points.append(p3)
                marker.points.append(p2)
                marker.points.append(p4)
    
        # Scale
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color (light blue, semi-transparent)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.3 # opacity

        self.plane_marker_pub.publish(marker)
        self.get_logger().info('Published water plane marker')


def main(args=None):
    rclpy.init(args=args)
    node = DeleteWaterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()