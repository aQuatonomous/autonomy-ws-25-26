# pointcloud_filters

Point cloud filtering and detection utilities for LiDAR streams.

## Nodes

### 1. `lidar_range_filter`
Filters raw LiDAR point clouds by range and vertical height.

- Subscribes: `/unilidar/cloud` (sensor_msgs/PointCloud2, frame: `unilidar_lidar`)
- Transforms points into `base_link` (via tf2) if enabled
- Clips by vertical height z ∈ [0, 10] m (in base_link)
- Clips by horizontal range ≤ 30 m
- Publishes: `/points_filtered` (sensor_msgs/PointCloud2)
- Buffers: stores up to 5 s of filtered clouds for windowed retrieval via service

#### Parameters
- `input_topic` (string, default `/unilidar/cloud`)
- `output_topic` (string, default `/points_filtered`)
- `base_frame` (string, default `base_link`)
- `use_tf_transform` (bool, default `False`)
- `z_min` (float, default `0.0`)
- `z_max` (float, default `10.0`)
- `range_max` (float, default `30.0`)
- `max_buffer_duration_sec` (float, default `5.0`) — maximum time window of clouds stored in buffer
- `keep_publisher` (bool, default `True`) — continue publishing `/points_filtered` for RViz/consumers

#### Launch
```bash
ros2 launch pointcloud_filters lidar_range_filter.launch.py
```

#### Service: `~/get_cloud_window`
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

---

### 2. `buoy_detector`
Detects buoys from filtered point clouds using DBSCAN clustering (hybrid Cartesian-Polar approach).

- Subscribes: `/points_filtered` (sensor_msgs/PointCloud2)
- Clusters points in Cartesian (x, y) space using Euclidean distance
- Converts cluster centroids to polar coordinates (range, bearing)
- Validates cluster size and publishes detections
- Publishes: `/buoy_detections` (pointcloud_filters/BuoyDetectionArray)

#### Parameters
- `input_topic` (string, default `/points_filtered`)
- `output_topic` (string, default `/buoy_detections`)
- `eps` (float, default `0.8`) — DBSCAN distance threshold in meters (KEY TUNING PARAMETER!)
- `min_samples` (int, default `5`) — minimum points to form a cluster
- `min_lateral_extent` (float, default `0.2`) — minimum buoy size in meters
- `max_lateral_extent` (float, default `3.0`) — maximum buoy size in meters  
- `min_points_final` (int, default `3`) — final validation point count
- `confidence_scale` (float, default `15.0`) — points needed for 100% confidence

#### Launch
```bash
ros2 launch pointcloud_filters buoy_detector.launch.py
```

#### Output Message: `BuoyDetection`
```
float32 range              # Distance from boat (meters, polar coordinate)
float32 bearing            # Angle from forward (radians, -π to +π, polar coordinate)
float32 z_mean             # Average height above base_link (meters)
uint32 num_points          # LiDAR points in cluster
float32 lateral_extent     # Bounding box diagonal (meters)
float32 radial_extent      # Depth toward/away from boat (meters)
float32 confidence         # Quality score 0.0-1.0 (higher = more points)
```

#### Tuning Guide
**Most important parameter: `eps` (distance threshold)**

- Start with `eps=0.8` (typical buoy diameter)
- If far buoys split into multiple clusters → increase to `1.0` or `1.2`
- If separate buoys merge together → decrease to `0.6` or `0.7`
- If water noise passes through → increase `min_samples` to `7` or `8`

**Test with real data:**
```bash
# Record bag with buoys at various ranges
ros2 bag record /unilidar/cloud -o buoy_test

# Launch full pipeline
ros2 launch unitree_lidar_ros2 launch.py  # LiDAR driver
ros2 launch pointcloud_filters lidar_range_filter.launch.py  # Filter
ros2 launch pointcloud_filters buoy_detector.launch.py  # Detector

# Monitor detections
ros2 topic echo /buoy_detections
```

---

## Algorithm: Hybrid Cartesian-Polar Clustering

The buoy detector uses a **hybrid approach**:
1. **Cluster in Cartesian (x, y)**: DBSCAN with Euclidean distance ensures physically meaningful clusters
2. **Output in Polar (range, bearing)**: Converts centroids for navigation/tracking downstream

**Why this works:**
- ✅ Cartesian clustering: Distance threshold has clear physical meaning (meters)
- ✅ Polar output: Matches navigation requirements (range, bearing)
- ✅ Best of both worlds!

---

## Notes
- Uses `sensor_msgs_py.point_cloud2` for PointCloud2 conversion (no ROS 1 deps).
- `buoy_detector` requires `python3-sklearn` (DBSCAN implementation)
- If TF lookup fails in range filter, node assumes cloud is already in `base_link`.
- If present, `intensity` is preserved in filtered clouds; other extra fields are omitted.
- Buffer is thread-safe (Lock-guarded) and automatically evicts old data beyond `max_buffer_duration_sec`.

Vendor defaults
- Cloud topic: `/unilidar/cloud` (frame: `unilidar_lidar`)
- IMU topic: `/unilidar/imu` (frame: `unilidar_imu`) — not used by this node directly, but documented for reference