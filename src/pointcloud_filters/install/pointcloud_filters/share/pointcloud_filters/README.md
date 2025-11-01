# pointcloud_filters

Point cloud filtering utilities for LiDAR streams. Currently provides a `lidar_range_filter` node that:

- Subscribes: `/unilidar/cloud` (sensor_msgs/PointCloud2, frame: `unilidar_lidar`)
- Transforms points into `base_link` (via tf2) if enabled
- Clips by vertical height z ∈ [0, 10] m (in base_link)
- Clips by horizontal range ≤ 30 m
- Publishes: `/points_filtered` (sensor_msgs/PointCloud2)

## Parameters
- `input_topic` (string, default `/unilidar/points_raw`)
- `output_topic` (string, default `/points_filtered`)
- `base_frame` (string, default `base_link`)
- `use_tf_transform` (bool, default `True`)
- `z_min` (float, default `0.0`)
- `z_max` (float, default `10.0`)
- `range_max` (float, default `30.0`)

## Launch
```
ros2 launch pointcloud_filters lidar_range_filter.launch.py
```

## Notes
- Uses `sensor_msgs_py.point_cloud2` for PointCloud2 conversion (no ROS 1 deps).
- If TF lookup fails, the node proceeds assuming the cloud is already expressed in `base_link`.
- If present, `intensity` is preserved; other extra fields are omitted in the filtered output.

Vendor defaults
- Cloud topic: `/unilidar/cloud` (frame: `unilidar_lidar`)
- IMU topic: `/unilidar/imu` (frame: `unilidar_imu`) — not used by this node directly, but documented for reference