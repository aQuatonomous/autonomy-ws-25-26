#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros

from sensor_msgs_py import point_cloud2 as pc2


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
        self.declare_parameter('z_max', 10.0)
        self.declare_parameter('range_max', 30.0)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.use_tf_transform = self.get_parameter('use_tf_transform').get_parameter_value().bool_value
        self.z_min = float(self.get_parameter('z_min').get_parameter_value().double_value)
        self.z_max = float(self.get_parameter('z_max').get_parameter_value().double_value)
        self.range_max = float(self.get_parameter('range_max').get_parameter_value().double_value)

        # TF buffer/listener for transforms to base_frame
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        if self.use_tf_transform:
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, qos)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)

        self.get_logger().info(
            f'lidar_range_filter started: input={self.input_topic}, output={self.output_topic}, '
            f'z=[{self.z_min},{self.z_max}] m, range_max={self.range_max} m, base_frame={self.base_frame}, '
            f'use_tf_transform={self.use_tf_transform}'
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
            fields = pc2.create_cloud_xyz32(header, [])
            self.pub.publish(fields)
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

        self.pub.publish(out_msg)


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
