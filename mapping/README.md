# mapping-25-26

ROS 2 perception pipeline for autonomous surface vehicle navigation using LiDAR-based buoy detection and tracking.

**Quick start:** `ros2 launch pointcloud_filters buoy_pipeline.launch.py` (from this folder after `source install/setup.bash`).  
**Competition and testing:** [COMPETITION.md](COMPETITION.md) — what to run when, overrides, inside-bay, tuning, troubleshooting.  
**WSL2 + USB LiDAR:** [WSL2_LIDAR_SETUP.md](WSL2_LIDAR_SETUP.md).

## Overview

This repository implements a multi-stage perception system that processes LiDAR point clouds to detect and track navigation buoys. The pipeline integrates with computer vision and planning systems to provide reliable landmark detection for autonomous navigation.

## Core Components

### 1. LiDAR Range Filter (`lidar_range_filter`)

Preprocesses raw LiDAR point clouds with rotations, FOV cone, height/range clipping, and optional TF and temporal accumulation.

- **Subscribes:** `input_topic` (default `/unilidar/cloud`, sensor_msgs/PointCloud2, frame `unilidar_lidar`) with BEST_EFFORT QoS to match the Unitree driver.
- **Publishes:** `output_topic` (default `/points_filtered`, sensor_msgs/PointCloud2).
- **Processing order (per frame):**
  1. Read points (x, y, z; intensity kept when present).
  2. **Rotation Z** (clockwise by `rotate_cw_deg`, default 202.5°).
  3. **Rotation Y** (counter-clockwise by `rotate_ccw_y_deg`, default 15°).
  4. **FOV filter:** Keep only points within `fov_max_angle_from_x` (default 105°) from the +X axis (cone in front of lidar).
  5. **Z ceiling:** Remove points with z > `z_max_cutoff` (default 1.5 m).
  6. **Optional:** Transform to `base_frame` via TF2 if `use_tf_transform` is true.
  7. **Range/z band:** Keep points with horizontal range ≤ `range_max`, and `z_min` ≤ z ≤ `z_max`.

- **Parameters:** `input_topic`, `output_topic`, `base_frame`, `use_tf_transform`, `z_min` (default -0.37), `z_max`, `range_max`, `max_buffer_duration_sec`, `keep_publisher`, `enable_accumulation` (default false), `accumulation_window` (default 0.2), `rotate_cw_deg`, `rotate_ccw_y_deg`, `fov_max_angle_from_x`, `z_max_cutoff`.

- **Service `~/get_cloud_window`** (pointcloud_filters/srv/GetCloudWindow): request `window_sec`, `merged`; returns windowed or merged clouds from the buffer.

### 2. Buoy Detector (`buoy_detector`)

Detects buoy candidates using **DBSCAN clustering** in Cartesian (x, y), then outputs polar (range, bearing) with validation.

- **Subscribes:** `input_topic` (default `/points_filtered`).
- **Publishes:** `output_topic` (default `/buoy_detections`, pointcloud_filters/BuoyDetectionArray). Publishes every frame (including empty) so the tracker and visualizers update promptly.
- **Algorithm:**
  1. Optionally remove water plane with RANSAC (`ransac_enabled`, default **false**).
  2. Cluster in (x, y) with DBSCAN (`eps`, `min_samples`).
  3. For each cluster: centroid → range and bearing; validate by lateral extent, point count, aspect ratio (only for clusters with >3 points), isolation, and external point count.
  4. Output detections with range, bearing, z_mean, num_points, lateral_extent, radial_extent, confidence.

- **Key parameters (defaults):** `eps` 0.8 m, `min_samples` 2, `min_lateral_extent` 0.01 m, `max_lateral_extent` 1.0 m, `min_points_final` 2, `min_isolation_margin` 0 (off), `max_aspect_ratio` 10.0, `max_external_points_nearby` -1 (off), `confidence_scale` 15.0, `ransac_enabled` false, `ransac_iterations` 80, `ransac_distance_threshold` 0.15, `ransac_min_inlier_ratio` 0.3.

- **Tuning:** Increase `eps` (e.g. 0.9–1.0) if small/sparse buoys don’t form one cluster; decrease if clusters merge. Use `buoy_detector_log_level:=debug` to see why clusters are rejected.

### 3. Buoy Tracker (`buoy_tracker`)

Maintains persistent buoy identities across frames with data association and a probation (candidate) phase.

- **Subscribes:** `/buoy_detections`.
- **Publishes:** `/tracked_buoys` (pointcloud_filters/TrackedBuoyArray).
- **Logic:** Nearest-neighbor association (distance threshold); unmatched detections become candidates; a candidate is promoted to a tracked buoy after being seen **min_observations_to_add** times in the same area; candidates dropped after **candidate_max_consecutive_misses** frames without match. Tracks are published only after **min_observations_for_publish**; removed after **max_consecutive_misses**. Class is not set here; the fusion node assigns class_id/class_name from CV to produce `/fused_buoys`.
- **Key parameters:** `association_threshold` (m), `min_observations_to_add`, `candidate_max_consecutive_misses`, `min_observations_for_publish`, `max_consecutive_misses`.

### 4. Tracked Buoy Visualizer (`tracked_buoy_visualizer`)

Visualizes fused buoys in RViz with cylinders and text labels (class_name from CV, e.g. red_buoy, red_pole_buoy).

- **Subscribes:** `/fused_buoys` (FusedBuoyArray). Run the fusion node so this topic is published.
- **Publishes:** `/tracked_buoy_markers` (visualization_msgs/MarkerArray): cylinder markers and, when `show_text_labels` is true, text showing **ID**, **distance (m)**, and **angle (°)**. Angle convention: **0° = +X**; above the X axis = negative angle, below = positive (i.e. angle = −degrees(bearing), bearing = atan2(y, x)).
- **Parameters:** `marker_height`, `marker_radius`, `marker_lifetime`, `show_text_labels` (default true).

## Pipeline Architecture

```
LiDAR driver (unitree_lidar_ros2)
    → /unilidar/cloud
    → lidar_range_filter (rotations, FOV, z ceiling, range/z band)
    → /points_filtered
    → buoy_detector (optional RANSAC, DBSCAN, validation)
    → /buoy_detections
    → buoy_tracker → /tracked_buoys (final LiDAR output, no color) and /tracked_buoys_json (same as JSON, similar to CV)
    → (fusion node: /tracked_buoys + /combined/detection_info_with_distance → /fused_buoys with class_id, class_name)
    → tracked_buoy_visualizer (/fused_buoys) → /tracked_buoy_markers (RViz)
```

## Final LiDAR output (downstream)

LiDAR does **not** set class; the fusion node assigns **class_id** and **class_name** from CV and publishes **`/fused_buoys`**.

| Topic | Type | Description |
|-------|------|-------------|
| **`/tracked_buoys`** | `pointcloud_filters/TrackedBuoyArray` | **Final LiDAR output**: persistent buoy tracks with `id`, `range` (m), `bearing` (rad), `x`, `y` (base_link), `z_mean`, `confidence`, `observation_count`, `first_seen`, `last_seen`. No class field. Published by `buoy_tracker` at detection rate (~10–30 Hz). |
| **`/tracked_buoys_json`** | `std_msgs/String` (JSON) | Same content as `/tracked_buoys` in JSON form (similar format to CV final output), published by **buoy_tracker**. `timestamp`, `frame_id`, `num_detections`, `detections[]` with `id`, `range`, `bearing`, `x`, `y`, `z_mean`, etc. |
| **`/fused_buoys`** | `pointcloud_filters/FusedBuoyArray` | **Output of CV–LiDAR fusion**: same geometry as `/tracked_buoys` with **`class_id`** and **`class_name`** from vision (e.g. `red_buoy`, `red_pole_buoy`, `green_buoy`, `yellow_buoy`, `unknown`). Run the fusion node (`cv_lidar_fusion`) subscribing to `/combined/detection_info_with_distance` and `/tracked_buoys`. **`detection_to_global_node`** subscribes to `/fused_buoys` when `use_fused_detections:=true` (default) and publishes `/global_detections` for planning. |

**Sample — `/tracked_buoys`** (no class):

```yaml
header: { stamp: {...}, frame_id: 'base_link' }
buoys:
  - id: 0
    range: 5.24
    bearing: 0.31
    x: 4.98
    y: 1.58
    z_mean: -0.12
    confidence: 0.82
    observation_count: 12
    first_seen: {...}
    last_seen: {...}
```

**`/fused_buoys`** has the same fields per buoy plus **`class_id`** (0–22 per `cv_scripts/class_mapping.yaml`; **255 = unknown** for unmatched LiDAR tracks) and **`class_name`** (e.g. `"red_buoy"`, `"red_pole_buoy"`, `"unknown"`). Planning receives `/global_detections` from `detection_to_global_node`, which feeds from `/fused_buoys` or `/tracked_buoys_json` based on `use_fused_detections`.

## Custom Messages

- **BuoyDetection:** `range`, `bearing`, `z_mean`, `num_points`, `lateral_extent`, `radial_extent`, `confidence`
- **BuoyDetectionArray:** Array of BuoyDetection
- **TrackedBuoy:** `id`, `range`, `bearing`, `x`, `y`, `z_mean`, `confidence`, `observation_count`, `first_seen`, `last_seen` (no class)
- **TrackedBuoyArray:** Array of TrackedBuoy
- **FusedBuoy:** TrackedBuoy fields plus **`class_id`** and **`class_name`** (from CV fusion)
- **FusedBuoyArray:** Array of FusedBuoy

## Launch

- **All-in-one (recommended):** `ros2 launch pointcloud_filters buoy_pipeline.launch.py` — driver, range filter, detector, tracker, visualizer, RViz. Launch defaults: `z_min` -0.37, `z_max` 10, `range_max` 30, `eps` 0.8, `min_samples` 2, `min_lateral_extent` 0.01, `ransac_enabled` false. See [COMPETITION.md](COMPETITION.md) for overrides and tuning.
- **Individual nodes:** `buoy_detector.launch.py`, or `ros2 run pointcloud_filters lidar_range_filter` / `buoy_detector` / `buoy_tracker` / `tracked_buoy_visualizer` with `--ros-args -p ...`.

## Dependencies

- ROS 2 (Humble or Iron)
- `sensor_msgs_py` (PointCloud2)
- `scikit-learn` (DBSCAN)
- `tf2_ros` (optional, for range filter transform)
- Unitree LiDAR driver (`unitree_lidar_ros2`) and `rviz2` for the full pipeline launch

## Monitoring and Debugging

### Basic topic monitoring

```bash
# List all topics
ros2 topic list

# Check topic frequency
ros2 topic hz /unilidar/cloud --window 20
ros2 topic hz /points_filtered --window 20
ros2 topic hz /buoy_detections --window 20
ros2 topic hz /tracked_buoys --window 20
ros2 topic hz /fused_buoys --window 20

# Check topic type
ros2 topic type /tracked_buoys
ros2 topic type /fused_buoys

# See topic info
ros2 topic info /tracked_buoys --verbose
ros2 topic info /fused_buoys --verbose
```

### Viewing topic messages

**Basic echo:**
```bash
ros2 topic echo /tracked_buoys
ros2 topic echo /fused_buoys
ros2 topic echo /buoy_detections
```

**View long messages (no truncation):**
```bash
# Use --no-arr to see full arrays/lists without truncation
ros2 topic echo /tracked_buoys --no-arr
ros2 topic echo /fused_buoys --no-arr
ros2 topic echo /buoy_detections --no-arr

# For JSON topics, extract and pretty-print with jq
ros2 topic echo /tracked_buoys_json --once --field data | jq .

# View full message continuously without truncation
ros2 topic echo /tracked_buoys_json --no-arr --field data | jq .
```

**Single message:**
```bash
# Get one message and exit
ros2 topic echo /tracked_buoys --once
ros2 topic echo /fused_buoys --once

# Single message, specific field, pretty-printed (for JSON topics)
ros2 topic echo /tracked_buoys_json --once --field data | jq .
```

**Point cloud topics:**
```bash
# Check point cloud data (use RViz for visualization)
ros2 topic echo /unilidar/cloud --once
ros2 topic echo /points_filtered --once

# Check point cloud frequency and size
ros2 topic hz /unilidar/cloud --window 20
ros2 topic hz /points_filtered --window 20
```

### Debugging specific nodes

**Check if nodes are running:**
```bash
ros2 node list
ros2 node info /lidar_range_filter
ros2 node info /buoy_detector
ros2 node info /buoy_tracker
ros2 node info /tracked_buoy_visualizer
```

**View node parameters:**
```bash
ros2 param list /buoy_detector
ros2 param get /buoy_detector eps
ros2 param get /buoy_detector min_samples
ros2 param get /buoy_tracker association_threshold
ros2 param get /buoy_tracker min_observations_to_add
```

**Monitor node output/logs:**
```bash
# Check if nodes are publishing
ros2 topic info /buoy_detections --verbose
ros2 topic info /tracked_buoys --verbose

# Enable debug logging for detector
ros2 launch pointcloud_filters buoy_pipeline.launch.py buoy_detector_log_level:=debug
```

### Common debugging commands

```bash
# Check all LiDAR-related topics
ros2 topic list | grep -E "(unilidar|points|buoy|tracked|fused)"

# Monitor pipeline health
ros2 topic hz /unilidar/cloud --window 20
ros2 topic hz /points_filtered --window 20
ros2 topic hz /buoy_detections --window 20
ros2 topic hz /tracked_buoys --window 20
ros2 topic hz /fused_buoys --window 20

# Save topic messages to bag file for later analysis
ros2 bag record /tracked_buoys /fused_buoys /buoy_detections /points_filtered

# Play back bag file
ros2 bag play <bag_file_name>
```

### RViz visualization

**Fixed frame:** `unilidar_lidar` (or `base_link` if TF transform is enabled)

**Topics to add in RViz:**
- PointCloud2: `/unilidar/cloud` (raw LiDAR)
- PointCloud2: `/points_filtered` (filtered cloud)
- MarkerArray: `/tracked_buoy_markers` (visualized buoys with labels)

**Marker labels show:**
- ID (track ID)
- Distance (m)
- Angle (°) where 0° = +X axis, above X = negative angle, below = positive

### Troubleshooting

- **No `/unilidar/cloud`:** Start the Unitree LiDAR driver; ensure device is connected (or use `launch_lidar_driver:=false` if cloud comes from elsewhere)
- **No `/points_filtered`:** Start the range filter after the driver. Check `range_max`, `z_min`, `z_max` parameters
- **No or bad buoy detections:** Tune `eps` (e.g. 0.8 → 1.0 if buoys split; 0.6 if they merge). Run with `buoy_detector_log_level:=debug` to see why clusters are rejected
- **Truncated output:** Use `--no-arr` flag with `ros2 topic echo` to see full messages
- **Wrong workspace:** Always `source install/setup.bash` from the **mapping** workspace (`~/autonomy-ws-25-26/mapping`)
- **CycloneDDS / rmw handle invalid:** Use FastRTPS: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` before launching
- **ModuleNotFoundError: sklearn:** `sudo apt install python3-sklearn`

For WSL2-specific issues (usbipd, serial port, RViz crashes), see [WSL2_LIDAR_SETUP.md](WSL2_LIDAR_SETUP.md). For competition-specific overrides and tuning, see [COMPETITION.md](COMPETITION.md).

## Notes

- Range filter uses BEST_EFFORT to match the Unitree driver; detector publishes every frame (including empty); tracker and visualizers use RELIABLE, depth 1 for low latency.
- DBSCAN in (x, y) gives physically meaningful distance; polar output (range, bearing) matches navigation. RViz labels show distance and angle (0° = +X, above X = negative).
- Thread-safe buffering in range filter with automatic eviction; intensity preserved when present in the cloud.
