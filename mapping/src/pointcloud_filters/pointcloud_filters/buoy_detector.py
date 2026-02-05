#!/usr/bin/env python3
"""
Buoy Detector Node - Uses DBSCAN clustering to detect buoys from filtered LiDAR point clouds.

Subscribes to: /points_filtered (sensor_msgs/PointCloud2)
Publishes to:  /buoy_detections (pointcloud_filters/BuoyDetectionArray)

Algorithm:
1. Cluster points using DBSCAN in Cartesian (x, y) coordinates
2. Extract cluster centroids and convert to polar (range, bearing)
3. Validate cluster size and publish detections
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from sklearn.cluster import DBSCAN

# Custom messages (will be generated)
from pointcloud_filters.msg import BuoyDetection, BuoyDetectionArray


class BuoyDetectorNode(Node):
    def __init__(self):
        super().__init__('buoy_detector')

        # Parameters
        self.declare_parameter('input_topic', '/points_filtered')
        self.declare_parameter('output_topic', '/buoy_detections')
        self.declare_parameter('eps', 0.8)  # DBSCAN distance threshold (meters)
        self.declare_parameter('min_samples', 5)  # DBSCAN minimum points per cluster
        self.declare_parameter('min_lateral_extent', 0.2)  # Minimum buoy size (meters)
        self.declare_parameter('max_lateral_extent', 3.0)  # Maximum buoy size (meters)
        self.declare_parameter('min_points_final', 3)  # Minimum points after validation
        self.declare_parameter('confidence_scale', 15.0)  # Points for 100% confidence
        
        # RANSAC water plane removal parameters
        self.declare_parameter('ransac_enabled', True)  # Enable RANSAC plane removal
        self.declare_parameter('ransac_iterations', 80)  # Number of RANSAC iterations (lower = less latency)
        self.declare_parameter('ransac_distance_threshold', 0.15)  # Max distance to plane (meters)
        self.declare_parameter('ransac_min_inlier_ratio', 0.3)  # Minimum fraction of points in plane

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.eps = self.get_parameter('eps').get_parameter_value().double_value
        self.min_samples = int(self.get_parameter('min_samples').get_parameter_value().integer_value)
        self.min_lateral_extent = self.get_parameter('min_lateral_extent').get_parameter_value().double_value
        self.max_lateral_extent = self.get_parameter('max_lateral_extent').get_parameter_value().double_value
        self.min_points_final = int(self.get_parameter('min_points_final').get_parameter_value().integer_value)
        self.confidence_scale = self.get_parameter('confidence_scale').get_parameter_value().double_value
        
        self.ransac_enabled = self.get_parameter('ransac_enabled').get_parameter_value().bool_value
        self.ransac_iterations = int(self.get_parameter('ransac_iterations').get_parameter_value().integer_value)
        self.ransac_distance_threshold = self.get_parameter('ransac_distance_threshold').get_parameter_value().double_value
        self.ransac_min_inlier_ratio = self.get_parameter('ransac_min_inlier_ratio').get_parameter_value().double_value

        # QoS profile matching LiDAR driver (best effort)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriber to filtered point cloud
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            qos
        )

        # Publisher for buoy detections (RELIABLE so tracker and visualizers can subscribe)
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(BuoyDetectionArray, self.output_topic, qos_pub)

        self.get_logger().info(
            f'Buoy detector started: input={self.input_topic}, output={self.output_topic}, '
            f'eps={self.eps}m, min_samples={self.min_samples}'
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        """Process incoming point cloud and detect buoys."""
        # Extract points from PointCloud2
        points = self.extract_points(msg)

        if len(points) == 0:
            self.get_logger().debug('Received empty point cloud')
            self.publish_detections([], msg.header)
            return

        # Remove water plane using RANSAC (if enabled)
        if self.ransac_enabled:
            points_filtered = self.remove_water_plane_ransac(points)
            if len(points_filtered) == 0:
                self.get_logger().debug('All points removed by RANSAC plane filtering')
                self.publish_detections([], msg.header)
                return
            points = points_filtered

        # Cluster points using DBSCAN
        labels = self.detect_buoys_dbscan(points)

        # Extract buoy detections from clusters
        buoys = self.extract_buoy_detections(points, labels)

        # Always publish (including empty) so tracker and visualizers update promptly
        self.publish_detections(buoys, msg.header)
        if len(buoys) > 0:
            self.get_logger().info(f'Detected {len(buoys)} buoy(s)')

    def extract_points(self, msg: PointCloud2) -> np.ndarray:
        """
        Extract XYZ points from PointCloud2 message.
        
        Returns:
            Nx3 numpy array of (x, y, z) coordinates
        """
        # Determine available fields
        field_map = {f.name: f for f in msg.fields}
        has_xyz = all(f in field_map for f in ['x', 'y', 'z'])
        
        if not has_xyz:
            self.get_logger().error('PointCloud2 missing x, y, or z fields')
            return np.array([])

        # Read points (handle mixed datatypes robustly)
        try:
            pts_iter = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as e:
            self.get_logger().error(f'Failed to read points: {e}')
            return np.array([])

        # Build list of (x, y, z) tuples
        values = []
        try:
            for p in pts_iter:
                # Handle both structured numpy records and plain sequences
                if hasattr(p, 'dtype') and p.dtype.names:
                    # Structured record (np.void)
                    values.append((float(p['x']), float(p['y']), float(p['z'])))
                else:
                    # Plain sequence
                    values.append((float(p[0]), float(p[1]), float(p[2])))
        except Exception as e:
            self.get_logger().error(f'Error parsing points: {e}')
            return np.array([])

        if len(values) == 0:
            return np.array([])

        # Convert to numpy array
        points = np.array(values, dtype=np.float64)
        return points

    def remove_water_plane_ransac(self, points: np.ndarray) -> np.ndarray:
        """
        Remove water plane using RANSAC plane fitting.
        
        Algorithm:
        1. Randomly sample 3 points to define a plane
        2. Count how many points are within distance_threshold of this plane (inliers)
        3. Repeat for ransac_iterations and keep the best plane (most inliers)
        4. Remove all inliers (water surface) and return outliers (buoys)
        
        A plane is defined by: n · p + d = 0
        where n = [a, b, c] is the unit normal vector and d is distance from origin.
        
        Args:
            points: Nx3 array (x, y, z)
        
        Returns:
            Filtered Nx3 array with water plane points removed
        """
        if len(points) < 3:
            return points
        
        best_inliers = None
        best_num_inliers = 0
        
        # RANSAC iterations
        for _ in range(self.ransac_iterations):
            # Step 1: Randomly sample 3 points
            indices = np.random.choice(len(points), 3, replace=False)
            sample_points = points[indices]
            
            # Step 2: Fit a plane through these 3 points
            # Plane is defined by two vectors in the plane
            v1 = sample_points[1] - sample_points[0]
            v2 = sample_points[2] - sample_points[0]
            
            # Normal vector = cross product of v1 and v2
            normal = np.cross(v1, v2)
            normal_length = np.linalg.norm(normal)
            
            # Skip if points are collinear (degenerate plane)
            if normal_length < 1e-6:
                continue
            
            # Normalize the normal vector
            n = normal / normal_length  # Unit normal [a, b, c]
            
            # Compute d: n · p0 + d = 0  =>  d = -n · p0
            d = -np.dot(n, sample_points[0])
            
            # Step 3: Count inliers (points close to this plane)
            # Distance from point p to plane: |n · p + d|
            distances = np.abs(np.dot(points, n) + d)
            inliers_mask = distances < self.ransac_distance_threshold
            num_inliers = np.sum(inliers_mask)
            
            # Update best plane if this one has more inliers
            if num_inliers > best_num_inliers:
                best_num_inliers = num_inliers
                best_inliers = inliers_mask
        
        # Check if we found a valid plane
        if best_inliers is None:
            self.get_logger().warn('RANSAC failed to find a plane')
            return points
        
        inlier_ratio = best_num_inliers / len(points)
        
        # Only remove plane if it has enough inliers (dominant surface = water)
        if inlier_ratio < self.ransac_min_inlier_ratio:
            self.get_logger().debug(
                f'RANSAC plane has {inlier_ratio:.1%} inliers (< {self.ransac_min_inlier_ratio:.1%} threshold), '
                'not removing - no dominant plane found'
            )
            return points
        
        # Remove inliers (water plane), keep outliers (buoys)
        filtered_points = points[~best_inliers]
        
        self.get_logger().info(
            f'RANSAC removed {best_num_inliers}/{len(points)} points '
            f'({inlier_ratio:.1%}) as water plane, {len(filtered_points)} remain'
        )
        
        return filtered_points

    def detect_buoys_dbscan(self, points: np.ndarray) -> np.ndarray:
        """
        Cluster LiDAR points into buoys using DBSCAN in Cartesian coordinates.
        
        Args:
            points: Nx3 array (x, y, z) in base_link frame
        
        Returns:
            labels: Cluster IDs for each point (-1 = noise)
        """
        if len(points) == 0:
            return np.array([])

        # Create DBSCAN clustering object
        # - Clusters in Cartesian (x, y) space
        # - Uses Euclidean distance metric
        # - Fast kd_tree algorithm for spatial indexing
        clustering = DBSCAN(
            eps=self.eps,              # Maximum distance for neighbors (meters)
            min_samples=self.min_samples,  # Minimum points for dense region
            metric='euclidean',        # Straight-line distance
            algorithm='kd_tree'        # Fast spatial search
        )

        # Cluster only in horizontal plane (x, y), ignore z
        # This prevents splitting a buoy vertically
        labels = clustering.fit_predict(points[:, :2])

        return labels

    def extract_buoy_detections(self, points: np.ndarray, labels: np.ndarray) -> list:
        """
        Convert DBSCAN clusters to buoy detections with polar coordinates.
        
        Args:
            points: Nx3 array (x, y, z) in Cartesian coordinates
            labels: Cluster IDs from DBSCAN
        
        Returns:
            List of buoy detection dicts with range, bearing, etc.
        """
        buoys = []
        unique_labels = set(labels)

        for label in unique_labels:
            # Skip noise points
            if label == -1:
                continue

            # Extract points belonging to this cluster
            cluster_mask = (labels == label)
            cluster_points = points[cluster_mask]

            # Compute centroid in Cartesian coordinates
            centroid_x = np.mean(cluster_points[:, 0])
            centroid_y = np.mean(cluster_points[:, 1])
            centroid_z = np.mean(cluster_points[:, 2])

            # Convert centroid to polar coordinates (range, bearing)
            # This is the hybrid approach: cluster in Cartesian, output in polar
            range_m = np.sqrt(centroid_x**2 + centroid_y**2)
            bearing_rad = np.arctan2(centroid_y, centroid_x)

            # Calculate cluster extents for validation
            # Radial extent (depth toward/away from boat)
            ranges = np.sqrt(cluster_points[:, 0]**2 + cluster_points[:, 1]**2)
            radial_extent = ranges.max() - ranges.min()

            # Lateral extent (bounding box diagonal)
            x_span = cluster_points[:, 0].max() - cluster_points[:, 0].min()
            y_span = cluster_points[:, 1].max() - cluster_points[:, 1].min()
            lateral_extent = np.sqrt(x_span**2 + y_span**2)

            # Validation filters
            # Reject if too large (probably shore/dock)
            if lateral_extent > self.max_lateral_extent:
                continue
            
            # Reject if too small (probably noise)
            if lateral_extent < self.min_lateral_extent:
                continue
            
            # Reject if too few points (safety check)
            if len(cluster_points) < self.min_points_final:
                continue

            # Compute confidence score (0.0 to 1.0)
            # More points = higher confidence, capped at 1.0
            confidence = min(1.0, len(cluster_points) / self.confidence_scale)

            # Package detection
            buoys.append({
                'range': range_m,
                'bearing': bearing_rad,
                'z_mean': centroid_z,
                'points': len(cluster_points),
                'lateral_extent_m': lateral_extent,
                'radial_extent_m': radial_extent,
                'confidence': confidence
            })

        return buoys

    def publish_detections(self, buoys: list, header) -> None:
        """Publish buoy detections as BuoyDetectionArray message."""
        msg = BuoyDetectionArray()
        msg.header = header
        msg.header.frame_id = header.frame_id  # Should be base_link or unilidar_lidar

        for buoy in buoys:
            detection = BuoyDetection()
            detection.range = float(buoy['range'])
            detection.bearing = float(buoy['bearing'])
            detection.z_mean = float(buoy['z_mean'])
            detection.num_points = int(buoy['points'])
            detection.lateral_extent = float(buoy['lateral_extent_m'])
            detection.radial_extent = float(buoy['radial_extent_m'])
            detection.confidence = float(buoy['confidence'])
            
            msg.detections.append(detection)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
