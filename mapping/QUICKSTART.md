# LiDAR pipeline – quick run & test

Short guide to run the LiDAR stack and check that you get correct output.

## Prerequisites

- ROS 2 (Humble or Iron) with workspace built.
- **Unitree LiDAR driver** (`unitree_lidar_ros2`) in the same workspace and built.  
  Raw LiDAR is published on `/unilidar/cloud` (frame: `unilidar_lidar`).
- Build the `mapping` workspace so `pointcloud_filters` is installed.

### Build (from the mapping folder)

```bash
cd /home/lorenzo/autonomy-ws-25-26/mapping
source /opt/ros/humble/setup.bash   # or iron
colcon build --packages-select unitree_lidar_ros2 pointcloud_filters --symlink-install
source install/setup.bash
```

**Note (lidar on Windows, dev in WSL2):** If the LiDAR is plugged into Windows and you run ROS 2 in WSL2, the USB device is not visible in WSL2 by default. Either attach the LiDAR to WSL2 (e.g. usbipd-win), or run the pipeline with the driver disabled:  
`ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_lidar_driver:=false`

---

## 1. Start the pipeline

### Option 1: One-command launch (recommended)

Starts LiDAR driver, range filter, buoy detector, tracker, visualizer, and RViz. **Single terminal**, from the mapping workspace:

```bash
cd /home/lorenzo/autonomy-ws-25-26/mapping
source install/setup.bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py
```

RViz opens with point clouds and tracked buoy markers. Fixed frame is `unilidar_lidar`.

**Launch defaults (match the code):** `z_min` -0.37, `z_max` 10, `range_max` 30, `eps` 0.8, `min_samples` 2, `min_lateral_extent` 0.01, `min_points_final` 2, `min_isolation_margin` 0, `max_external_points_nearby` -1, `max_aspect_ratio` 10, `ransac_enabled` false. Each buoy label in RViz shows **ID**, **distance (m)**, and **angle (°)** (0° = +X; above X = negative, below X = positive).

**Useful overrides:**

| Override | Example | Use case |
|----------|---------|----------|
| No LiDAR driver | `launch_lidar_driver:=false` | Driver runs elsewhere |
| No RViz | `launch_rviz:=false` | Headless or your own RViz |
| Short range / inside-bay | `range_max:=0.5 z_max:=0.5` | Close buoys |
| Looser clustering (small buoys) | `eps:=0.9` or `eps:=1.0` | Points farther apart still form one cluster |
| Stricter tracking | `min_observations_to_add:=8 candidate_max_consecutive_misses:=3` | Fewer false tracks |
| Debug detector rejections | `buoy_detector_log_level:=debug` | See why clusters are rejected |

**Buoy detection tuning – test commands**

- **eps** = max distance (m) between points in the same cluster; larger = looser.
- **min_samples** = DBSCAN min points to form a cluster (default 2).
- **min_points_final** = min points per cluster after validation (default 2).
- **min_lateral_extent** = min cluster size in m (default 0.01).

```bash
# Defaults (z_min=-0.37, ransac_enabled=false already)
ros2 launch pointcloud_filters buoy_pipeline.launch.py

# Looser clustering
ros2 launch pointcloud_filters buoy_pipeline.launch.py eps:=0.9
ros2 launch pointcloud_filters buoy_pipeline.launch.py eps:=1.0

# Tighter clustering (if buoys merge or big false clusters)
ros2 launch pointcloud_filters buoy_pipeline.launch.py eps:=0.6

# Inside-bay style (short range + tighter clusters)
ros2 launch pointcloud_filters buoy_pipeline.launch.py range_max:=0.5 z_max:=0.5 eps:=0.15 min_samples:=15
```

**Tracker tuning:** Increase `min_observations_to_add` (default 5) for fewer false buoys; decrease `candidate_max_consecutive_misses` (default 4) to drop candidates faster. Decrease `association_threshold` (default 0.5 m) for stricter matching.

**Requirements:** `unitree_lidar_ros2` and `rviz2` in the same workspace for the full launch.

---

### Option 2: Run nodes separately (4 terminals)

All terminals: `source install/setup.bash` (from mapping workspace).

**Terminal 1 – LiDAR driver**

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

Check: `ros2 topic hz /unilidar/cloud`

**Terminal 2 – Range filter**

No dedicated launch file; use `ros2 run` with parameters.

```bash
ros2 run pointcloud_filters lidar_range_filter --ros-args \
  -p input_topic:=/unilidar/cloud \
  -p output_topic:=/points_filtered \
  -p z_min:=-0.37 \
  -p z_max:=10.0 \
  -p range_max:=30.0 \
  -p use_tf_transform:=false
```

Short range (e.g. inside-bay): use `z_max:=0.5`, `range_max:=0.5`. Optional accumulation: `-p enable_accumulation:=true -p accumulation_window:=0.2`.

Check: `ros2 topic hz /points_filtered`

**Terminal 3 – Buoy detector**

```bash
ros2 launch pointcloud_filters buoy_detector.launch.py
```

Or with overrides: `ros2 run pointcloud_filters buoy_detector --ros-args -p eps:=0.8 -p min_samples:=2`

Check: `ros2 topic echo /buoy_detections`

**Terminal 4 – Tracker + visualizer**

```bash
ros2 run pointcloud_filters buoy_tracker --ros-args -p min_observations_for_publish:=2
ros2 run pointcloud_filters tracked_buoy_visualizer --ros-args -p show_text_labels:=true
```

Then start RViz and add the topics below.

---

## 2. Check that output is correct

| Check | Command / action |
|-------|-------------------|
| Raw LiDAR | `ros2 topic hz /unilidar/cloud` |
| Filtered cloud | `ros2 topic hz /points_filtered` |
| Detections | `ros2 topic echo /buoy_detections` |
| Tracked buoys | `ros2 topic echo /tracked_buoys` |

**RViz**

1. Run `rviz2`, set **Fixed Frame** to `unilidar_lidar`.
2. Add by topic:
   - **PointCloud2:** `/unilidar/cloud` (raw), `/points_filtered` (filtered).
   - **MarkerArray:** `/tracked_buoy_markers` (cylinders + labels with ID, distance m, angle °).

Labels use angle 0° = +X; above X = negative angle, below X = positive.

---

## 3. Typical issues

- **No `/unilidar/cloud`** — Start the Unitree LiDAR driver; ensure device is connected.
- **No `/points_filtered`** — Start the range filter after the driver. Check `range_max`, `z_min`, `z_max`.
- **No or bad buoy detections** — Tune `eps` (e.g. 0.8 → 1.0 if buoys split; 0.6 if they merge). For small buoys, defaults (eps 0.8, min_samples 2, min_lateral_extent 0.01) are already relaxed.
- **Buoys visible in RViz but 0 detections** — The detector logs a throttled hint (~5 s): “DBSCAN found no clusters” (try larger `eps` or smaller `min_samples`) or “N clusters rejected” (run with `buoy_detector_log_level:=debug` to see why).
- **Wrong workspace** — Always `source install/setup.bash` from the workspace that contains `pointcloud_filters` and `unitree_lidar_ros2`.

---

## 4. Low-latency tips

- **Range filter:** `enable_accumulation` is false by default for responsive output. Enable only when you need a denser merged cloud and accept delay.
- **Tracker:** For new buoys to appear sooner, set `min_observations_for_publish:=1` or `2` (default 3 reduces flicker).
- **RANSAC:** Off by default. Enable with `ransac_enabled:=true` if you need water-plane removal; it can remove low-lying buoy points.

---

## 5. Pipeline summary

```
LiDAR driver (unitree_lidar_ros2)
    → /unilidar/cloud
    → lidar_range_filter (rotations, FOV, z ceiling, range/z)
    → /points_filtered
    → buoy_detector (optional RANSAC, DBSCAN, validation)
    → /buoy_detections
    → buoy_tracker
    → /tracked_buoys
    → tracked_buoy_visualizer
    → /tracked_buoy_markers (RViz: cylinders + ID, distance m, angle °)
```

For full node and parameter details, see [README.md](README.md).
