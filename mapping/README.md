# mapping-25-26

ROS 2 perception pipeline for autonomous surface vehicle navigation using LiDAR-based buoy detection and tracking.

## Overview

This repository implements a multi-stage perception system that processes LiDAR point clouds to detect and track navigation buoys. The pipeline integrates with computer vision and planning systems to provide reliable landmark detection for autonomous navigation.

**Quick start:** See [QUICKSTART.md](QUICKSTART.md) to run the full pipeline (LiDAR driver, filter, detector, tracker, visualizer, RViz) in one command:  
`ros2 launch pointcloud_filters buoy_pipeline.launch.py`

## Core Components

### 1. LiDAR Range Filter (`lidar_range_filter`)
Preprocesses raw LiDAR point clouds with spatial filtering and optional temporal accumulation.

- **Subscribes:** `/unilidar/cloud` (sensor_msgs/PointCloud2, frame `unilidar_lidar`) with BEST_EFFORT QoS to match the Unitree driver
- **Publishes:** `/points_filtered` (sensor_msgs/PointCloud2)
- **Filtering:** Range-based (horizontal distance) and height-based (z-axis) clipping
- **Transform:** Optional TF2 transformation to `base_frame` (default `base_link`)
- **Buffering:** Time-windowed cloud storage (up to `max_buffer_duration_sec`) with service-based retrieval
- **Accumulation:** Optional temporal merging; off by default for low latency (use only when you need denser clouds)

**Parameters:** `input_topic`, `output_topic`, `base_frame`, `use_tf_transform`, `z_min`, `z_max`, `range_max`, `max_buffer_duration_sec`, `keep_publisher`, `enable_accumulation` (default `False`), `accumulation_window` (default `0.2`).

**Service `~/get_cloud_window`** (pointcloud_filters/srv/GetCloudWindow): request `window_sec`, `merged`; returns windowed or merged clouds from the buffer.

### 2. Buoy Detector (`buoy_detector`)
Detects buoy candidates using **DBSCAN clustering** with a hybrid Cartesian–polar approach.

- **Subscribes:** `/points_filtered`
- **Publishes:** `/buoy_detections` (pointcloud_filters/BuoyDetectionArray); publishes every frame (including empty) so tracker and visualizers update promptly
- **Algorithm:** Cluster in Cartesian (x, y) with Euclidean distance; convert centroids to polar (range, bearing); validate by size and point count
- **RANSAC:** Optional water-plane removal (`ransac_enabled`, `ransac_iterations` default 80)

**Key parameters:** `eps` (DBSCAN distance in m; primary tuning), `min_samples`, `min_lateral_extent`, `max_lateral_extent`, `min_points_final`, `confidence_scale`, `ransac_*`.

**Tuning `eps`:** Start at 0.8; increase to 1.0–1.2 if buoys split; decrease to 0.6–0.7 if buoys merge; increase `min_samples` to reduce water noise.

### 3. Buoy Tracker (`buoy_tracker`)
Maintains persistent buoy identities across frames using data association and a **probation (candidate)** phase so only stable objects become tracks.

- **Subscribes:** `/buoy_detections`
- **Publishes:** `/tracked_buoys` (pointcloud_filters/TrackedBuoyArray)
- **Probation:** Unmatched detections become "candidates"; a candidate is promoted to a tracked buoy only after it has been seen **min_observations_to_add** times in the same area (within **association_threshold**). Candidates that are not re-seen for **candidate_max_consecutive_misses** frames are dropped.
- Nearest-neighbor association (3D distance); exponential smoothing; **min_observations_for_publish** before a track is published; removal after **max_consecutive_misses**; ready for vision fusion (color/class).
- **Key parameters:** `association_threshold` (m), `min_observations_to_add` (stricter = fewer false buoys), `candidate_max_consecutive_misses`, `max_consecutive_misses`, `min_observations_for_publish`.

## Integration Points

### Computer Vision
- **Input**: Camera-based buoy detections with bearing hypotheses
- **Fusion**: LiDAR verifies range and validates visual detections
- **Output**: Tracked buoys with stable IDs for color/class annotation

### Planning
- **Localization**: Integrates with GPS/IMU fusion (MAVROS + `robot_localization`)
- **Mapping**: Converts detections to global map frame via TF2
- **Navigation**: Publishes buoy positions in both map and base_link frames

## Pipeline Architecture

```
LiDAR Driver → Range Filter → Buoy Detector → Buoy Tracker → Map Frame Projection
                    ↓              ↓               ↓
              /points_filtered  /buoy_detections  /tracked_buoys
```

## Custom Messages

- **BuoyDetection:** `range`, `bearing`, `z_mean`, `num_points`, `lateral_extent`, `radial_extent`, `confidence`
- **BuoyDetectionArray:** Array of detections from DBSCAN
- **TrackedBuoy:** `id`, `range`, `bearing`, `x`, `y`, `z_mean`, `color`, `confidence`, `observation_count`, `first_seen`, `last_seen`
- **TrackedBuoyArray:** Array of tracked buoys

## Launch

- **All-in-one (recommended):** `ros2 launch pointcloud_filters buoy_pipeline.launch.py` — driver, filter, detector, tracker, visualizer, RViz. See [QUICKSTART.md](QUICKSTART.md) for overrides (e.g. inside-bay, no RViz).
- **Individual nodes:** `buoy_detector.launch.py`, or `ros2 run pointcloud_filters lidar_range_filter` / `buoy_tracker` / `tracked_buoy_visualizer` with `--ros-args -p ...`.

## Dependencies

- ROS 2 (Humble/Iron)
- `sensor_msgs_py` (PointCloud2 utilities)
- `scikit-learn` (DBSCAN)
- `tf2_ros` (coordinate transformations)
- Unitree LiDAR driver (`unitree_lidar_ros2`) and `rviz2` for the full pipeline launch

## Notes

- Range filter subscribes with BEST_EFFORT to match the Unitree driver; detector publishes every frame (including empty); tracker and visualizers use QoS depth=1 for low latency.
- DBSCAN in Cartesian space gives physically meaningful distance thresholds; polar output matches navigation.
- Thread-safe buffering with automatic eviction; intensity preserved in filtered clouds when present.
- Vendor defaults: cloud topic `/unilidar/cloud` (frame `unilidar_lidar`), IMU `/unilidar/imu` (frame `unilidar_imu`).
