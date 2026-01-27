#!/usr/bin/env python3

import math
from typing import Optional
from collections import deque
from threading import Lock

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros

from sensor_msgs_py import point_cloud2 as pc2

from pointcloud_filters.srv import GetCloudWindow


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    # Normalize to be safe
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0.0:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    # Quaternion to rotation matrix
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    R = np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),         2 * (xz + wy)],
        [2 * (xy + wz),         1 - 2 * (xx + zz),     2 * (yz - wx)],
        [2 * (xz - wy),         2 * (yz + wx),         1 - 2 * (xx + yy)],
    ], dtype=np.float64)
    return R


class LidarRangeFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('lidar_range_filter')

        # Parameters
        # Default input topic from unilidar_sdk
        self.declare_parameter('input_topic', '/unilidar/cloud')
        self.declare_parameter('output_topic', '/points_filtered')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_tf_transform', False)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 0.5)
        self.declare_parameter('range_max', 0.5)
        self.declare_parameter('max_buffer_duration_sec', 5.0)
        self.declare_parameter('keep_publisher', True)
        self.declare_parameter('enable_accumulation', True)
        self.declare_parameter('accumulation_window', 1.0)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.use_tf_transform = self.get_parameter('use_tf_transform').get_parameter_value().bool_value
        self.z_min = float(self.get_parameter('z_min').get_parameter_value().double_value)
        self.z_max = float(self.get_parameter('z_max').get_parameter_value().double_value)
        self.range_max = float(self.get_parameter('range_max').get_parameter_value().double_value)
        self.max_buffer_duration_sec = float(self.get_parameter('max_buffer_duration_sec').get_parameter_value().double_value)
        self.keep_publisher = self.get_parameter('keep_publisher').get_parameter_value().bool_value
        self.enable_accumulation = self.get_parameter('enable_accumulation').get_parameter_value().bool_value
        self.accumulation_window = float(self.get_parameter('accumulation_window').get_parameter_value().double_value)

        # Time-ordered buffer for filtered clouds (stores up to max_buffer_duration_sec)
        self.cloud_buffer = deque()
        self.buffer_lock = Lock()

        # TF buffer/listener for transforms to base_frame
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        if self.use_tf_transform:
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, qos)
        
        # Publisher (conditional on keep_publisher param)
        if self.keep_publisher:
            self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        else:
            self.pub = None

        # Service for retrieving windowed point cloud data
        self.srv = self.create_service(GetCloudWindow, '~/get_cloud_window', self.handle_get_cloud_window)

        self.get_logger().info(
            f'lidar_range_filter started: input={self.input_topic}, output={self.output_topic if self.keep_publisher else "(disabled)"}, '
            f'z=[{self.z_min},{self.z_max}] m, range_max={self.range_max} m, base_frame={self.base_frame}, '
            f'use_tf_transform={self.use_tf_transform}, max_buffer_duration_sec={self.max_buffer_duration_sec}, '
            f'accumulation: {"enabled (" + str(self.accumulation_window) + "s)" if self.enable_accumulation else "disabled"}'
        )

    def lookup_transform(self, target_frame: str, source_frame: str, stamp) -> Optional[TransformStamped]:
        if not self.use_tf_transform or self.tf_buffer is None:
            return None
        try:
            # Use time from message; allow small timeout
            return self.tf_buffer.lookup_transform(target_frame, source_frame, stamp, timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed {source_frame} -> {target_frame} at time {stamp.sec}.{stamp.nanosec}: {e}')
            return None

    def cloud_cb(self, msg: PointCloud2) -> None:
        # Determine fields and read points (keep intensity if available)
        field_map = {f.name: f for f in msg.fields}
        keep_intensity = 'intensity' in field_map
        read_fields = ('x', 'y', 'z', 'intensity') if keep_intensity else ('x', 'y', 'z')

        # Always use read_points() to support mixed datatypes and padding in PointCloud2
        try:
            pts_iter = pc2.read_points(msg, field_names=read_fields, skip_nans=True)
        except Exception as e:
            self.get_logger().error(f'Failed to read points from PointCloud2: {e}')
            return

        # Build a list of numeric tuples robustly (supports structured records and sequences)
        values = []
        try:
            for p in pts_iter:
                # Structured numpy scalar (np.void) with named fields
                if isinstance(p, np.void) and getattr(p, 'dtype', None) is not None and p.dtype.fields is not None:
                    if keep_intensity:
                        values.append((float(p['x']), float(p['y']), float(p['z']), float(p['intensity'])))
                    else:
                        values.append((float(p['x']), float(p['y']), float(p['z'])))
                else:
                    # Assume indexable sequence/tuple
                    if keep_intensity:
                        values.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
                    else:
                        values.append((float(p[0]), float(p[1]), float(p[2])))
        except Exception as e:
            self.get_logger().error(f'Error parsing PointCloud2 fields: {e}')
            return

        if len(values) == 0:
            return

        # Convert to ndarray (float64 for math stability)
        pts = np.array(values, dtype=np.float64)

        if pts.size == 0:
            return

        x = pts[:, 0].astype(np.float64, copy=False)
        y = pts[:, 1].astype(np.float64, copy=False)
        z = pts[:, 2].astype(np.float64, copy=False)

        # Transform to base_frame if requested and frames differ
        src_frame = msg.header.frame_id
        transformed = False
        if self.use_tf_transform and src_frame and src_frame != self.base_frame:
            tf = self.lookup_transform(self.base_frame, src_frame, msg.header.stamp)
            if tf is not None:
                t = tf.transform.translation
                q = tf.transform.rotation
                R = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
                T = np.array([t.x, t.y, t.z], dtype=np.float64)
                pts = np.vstack((x, y, z))  # 3xN
                pts_tf = (R @ pts) + T.reshape(3, 1)
                x, y, z = pts_tf[0, :], pts_tf[1, :], pts_tf[2, :]
                transformed = True
            else:
                # Proceed without transform; assume already in base_frame
                pass

        # Compute range in the horizontal plane and apply filters
        rng = np.sqrt(x * x + y * y)
        mask = (rng <= self.range_max) & (z >= self.z_min) & (z <= self.z_max)
        header = msg.header
        if self.use_tf_transform:
            # Only relabel to base_frame if transform actually applied; otherwise keep source frame
            header.frame_id = self.base_frame if transformed else (src_frame or self.base_frame)
        else:
            header.frame_id = src_frame or self.base_frame

        if not np.any(mask):
            # Publish empty XYZ cloud
            out_msg = pc2.create_cloud_xyz32(header, [])
            if self.pub:
                self.pub.publish(out_msg)
            # Store empty cloud in buffer
            with self.buffer_lock:
                self._append_to_buffer(out_msg)
            return

        if keep_intensity:
            # Keep intensity; pack points as tuples (x,y,z,intensity)
            intens = pts[:, 3]
            sel = mask
            points_list = list(map(lambda i: (float(x[i]), float(y[i]), float(z[i]), float(intens[i])), np.flatnonzero(sel)))
            from sensor_msgs.msg import PointField
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            out_msg = pc2.create_cloud(header, fields, points_list)
        else:
            # XYZ only
            sel_pts = np.vstack((x[mask], y[mask], z[mask])).T.astype(np.float32)
            out_msg = pc2.create_cloud_xyz32(header, sel_pts.tolist())

        # Store filtered cloud in buffer first
        with self.buffer_lock:
            self._append_to_buffer(out_msg)
            
            # Publish accumulated or single frame based on settings
            if self.pub:
                if self.enable_accumulation:
                    # Get clouds within accumulation window and merge them
                    accumulated_cloud = self._get_accumulated_cloud()
                    self.pub.publish(accumulated_cloud)
                else:
                    # Publish just the current frame
                    self.pub.publish(out_msg)

    def _append_to_buffer(self, cloud: PointCloud2) -> None:
        """Append cloud to buffer and evict old entries beyond max_buffer_duration_sec."""
        self.cloud_buffer.append(cloud)
        # Evict old clouds
        if len(self.cloud_buffer) > 1:
            newest_stamp = self.cloud_buffer[-1].header.stamp
            newest_time_sec = newest_stamp.sec + newest_stamp.nanosec * 1e-9
            while len(self.cloud_buffer) > 1:
                oldest_stamp = self.cloud_buffer[0].header.stamp
                oldest_time_sec = oldest_stamp.sec + oldest_stamp.nanosec * 1e-9
                if (newest_time_sec - oldest_time_sec) > self.max_buffer_duration_sec:
                    self.cloud_buffer.popleft()
                else:
                    break
    
    def _get_accumulated_cloud(self) -> PointCloud2:
        """Get merged cloud from points within accumulation_window. Must be called with buffer_lock held."""
        if len(self.cloud_buffer) == 0:
            return PointCloud2()
        
        if len(self.cloud_buffer) == 1:
            return self.cloud_buffer[0]
        
        # Get newest timestamp and calculate cutoff
        newest_stamp = self.cloud_buffer[-1].header.stamp
        newest_time_sec = newest_stamp.sec + newest_stamp.nanosec * 1e-9
        cutoff_time_sec = newest_time_sec - self.accumulation_window
        
        # Select clouds within accumulation window
        selected = []
        for cloud in self.cloud_buffer:
            stamp = cloud.header.stamp
            time_sec = stamp.sec + stamp.nanosec * 1e-9
            if time_sec >= cutoff_time_sec:
                selected.append(cloud)
        
        if len(selected) == 0:
            return PointCloud2()
        
        # Merge and return
        return self._merge_clouds(selected)

    def handle_get_cloud_window(self, request, response):
        """Service handler: return windowed point clouds."""
        window_sec = min(request.window_sec, self.max_buffer_duration_sec)
        
        with self.buffer_lock:
            if len(self.cloud_buffer) == 0:
                # Empty buffer
                response.clouds = []
                response.merged_cloud = PointCloud2()
                return response

            # Get newest timestamp
            newest_stamp = self.cloud_buffer[-1].header.stamp
            newest_time_sec = newest_stamp.sec + newest_stamp.nanosec * 1e-9
            cutoff_time_sec = newest_time_sec - window_sec

            # Select clouds within window
            selected = []
            for cloud in self.cloud_buffer:
                stamp = cloud.header.stamp
                time_sec = stamp.sec + stamp.nanosec * 1e-9
                if time_sec >= cutoff_time_sec:
                    selected.append(cloud)

            if len(selected) == 0:
                response.clouds = []
                response.merged_cloud = PointCloud2()
                return response

            if request.merged:
                # Merge all selected clouds into one
                response.merged_cloud = self._merge_clouds(selected)
                response.clouds = []
            else:
                # Return array of individual clouds
                response.clouds = selected
                response.merged_cloud = PointCloud2()

        return response

    def _merge_clouds(self, clouds):
        """Concatenate multiple PointCloud2 messages into a single cloud."""
        if len(clouds) == 0:
            return PointCloud2()
        if len(clouds) == 1:
            return clouds[0]

        # Use the header from the newest cloud
        merged_header = clouds[-1].header

        # Determine if we have intensity field by checking first cloud
        first_cloud_fields = [f.name for f in clouds[0].fields]
        has_intensity = 'intensity' in first_cloud_fields
        
        # Read all points from all clouds, ensuring consistent field structure
        all_points = []
        for cloud in clouds:
            cloud_fields = [f.name for f in cloud.fields]
            cloud_has_intensity = 'intensity' in cloud_fields
            
            # Read with appropriate fields
            if has_intensity and cloud_has_intensity:
                field_names = ('x', 'y', 'z', 'intensity')
            else:
                field_names = ('x', 'y', 'z')
            
            try:
                pts_iter = pc2.read_points(cloud, field_names=field_names, skip_nans=True)
                for p in pts_iter:
                    if isinstance(p, np.void) and getattr(p, 'dtype', None) is not None and p.dtype.fields is not None:
                        if has_intensity and len(field_names) == 4:
                            all_points.append((float(p['x']), float(p['y']), float(p['z']), float(p['intensity'])))
                        else:
                            all_points.append((float(p['x']), float(p['y']), float(p['z'])))
                    else:
                        # Tuple/sequence format
                        if has_intensity and len(p) >= 4:
                            all_points.append((float(p[0]), float(p[1]), float(p[2]), float(p[3])))
                        else:
                            all_points.append((float(p[0]), float(p[1]), float(p[2])))
            except Exception as e:
                self.get_logger().warn(f'Failed to read points during merge: {e}')
                continue

        if len(all_points) == 0:
            return PointCloud2()

        # Reconstruct PointCloud2 with merged points using consistent field structure
        if has_intensity:
            from sensor_msgs.msg import PointField
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            merged_cloud = pc2.create_cloud(merged_header, fields, all_points)
        else:
            # XYZ only
            merged_cloud = pc2.create_cloud_xyz32(merged_header, all_points)

        return merged_cloud


def main(args=None):
    rclpy.init(args=args)
    node = LidarRangeFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
