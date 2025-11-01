# pointcloud_filters

Point cloud filtering utilities for LiDAR streams. Currently provides a `lidar_range_filter` node that:

- Subscribes: `/unilidar/cloud` (sensor_msgs/PointCloud2, frame: `unilidar_lidar`)
- Transforms points into `base_link` (via tf2) if enabled
- Clips by vertical height z ∈ [0, 10] m (in base_link)
- Clips by horizontal range ≤ 30 m
- Publishes: `/points_filtered` (sensor_msgs/PointCloud2)
- Buffers: stores up to 5 s of filtered clouds for windowed retrieval via service

## Parameters
- `input_topic` (string, default `/unilidar/cloud`)
- `output_topic` (string, default `/points_filtered`)
- `base_frame` (string, default `base_link`)
- `use_tf_transform` (bool, default `False`)
- `z_min` (float, default `0.0`)
- `z_max` (float, default `10.0`)
- `range_max` (float, default `30.0`)
- `max_buffer_duration_sec` (float, default `5.0`) — maximum time window of clouds stored in buffer
- `keep_publisher` (bool, default `True`) — continue publishing `/points_filtered` for RViz/consumers

## Launch
```
ros2 launch pointcloud_filters lidar_range_filter.launch.py
```

## Service: `~/get_cloud_window`
- Type: `pointcloud_filters/srv/GetCloudWindow`
- Description: retrieve a time window of filtered LiDAR clouds from the internal buffer
- Request:
  - `window_sec` (float32): duration of history to retrieve (0.0 to max_buffer_duration_sec)
  - `merged` (bool): if `true`, return a single concatenated cloud; if `false`, return an array of individual clouds
- Response:
  - `clouds` (sensor_msgs/PointCloud2[]): array of individual clouds (if merged=false)
  - `merged_cloud` (sensor_msgs/PointCloud2): single concatenated cloud (if merged=true)

Example call:
```bash
ros2 service call /lidar_range_filter/get_cloud_window pointcloud_filters/srv/GetCloudWindow "{window_sec: 2.5, merged: true}"
```

## Notes
- Uses `sensor_msgs_py.point_cloud2` for PointCloud2 conversion (no ROS 1 deps).
- If TF lookup fails, the node proceeds assuming the cloud is already expressed in `base_link`.
- If present, `intensity` is preserved; other extra fields are omitted in the filtered output.
- Buffer is thread-safe (Lock-guarded) and automatically evicts old data beyond `max_buffer_duration_sec`.

Vendor defaults
- Cloud topic: `/unilidar/cloud` (frame: `unilidar_lidar`)
- IMU topic: `/unilidar/imu` (frame: `unilidar_imu`) — not used by this node directly, but documented for reference