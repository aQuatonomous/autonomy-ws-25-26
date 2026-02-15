# LiDAR pipeline – quick run & test

Short guide to run the LiDAR stack and check that you get correct output.

## Prerequisites

- ROS 2 (Humble or Iron) with workspace built.
- **Unitree LiDAR driver** (`unitree_lidar_ros2`) in the same workspace and built.  
  Raw LiDAR is published on `/unilidar/cloud` (frame: `unilidar_lidar`).
- From this repo: build the `mapping` workspace so `pointcloud_filters` is installed.

### Build (from the mapping folder)

```bash
cd /home/lorenzo/autonomy-ws-25-26/mapping
source /opt/ros/humble/setup.bash   # or iron
colcon build --packages-select unitree_lidar_ros2 pointcloud_filters --symlink-install
source install/setup.bash
```

**Note (lidar on Windows, dev in WSL2):** If the LiDAR is plugged into your Windows laptop and you run ROS 2 in WSL2, the USB device is not visible in WSL2 by default. Either:
- **Option A:** Attach the LiDAR to WSL2 using [usbipd-win](https://github.com/dorssel/usbipd-win) (on Windows: `usbipd list`, then `usbipd bind -b <BUSID>` and in WSL2: `usbipd attach -w -b <BUSID>`), then run the full pipeline in WSL2; or  
- **Option B:** Run the pipeline with the driver disabled and no point cloud until the device is available in WSL2:  
  `ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_lidar_driver:=false`

---

## 1. Start the pipeline

### Option 1: One-command launch (recommended)

Starts LiDAR driver, range filter, buoy detector, tracker, visualizer, and RViz in one go. **Single terminal**, from the mapping workspace:

```bash
cd /home/lorenzo/autonomy-ws-25-26/mapping
source install/setup.bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py
```

RViz opens with raw cloud, filtered cloud, and tracked buoy markers. Fixed frame is `unilidar_lidar`.

**Useful overrides:**

| Override | Example | Use case |
|----------|---------|----------|
| No LiDAR driver (already running) | `launch_lidar_driver:=false` | Driver in another terminal |
| No RViz | `launch_rviz:=false` | Headless or your own RViz |
| Inside-bay / short range | `range_max:=0.5 z_max:=0.5` | Close buoys |
| Inside-bay detector tuning | `range_max:=0.5 z_max:=0.5 eps:=0.15 min_samples:=15` | Tighter clusters |
| New buoys appear sooner | `min_observations_for_publish:=1` | Less delay, possible flicker |
| **Stricter tracking (fewer false buoys)** | `min_observations_to_add:=8 candidate_max_consecutive_misses:=3` | Object must be seen more times in same area before becoming a track |
| Looser association | `association_threshold:=0.8` | Match detections to tracks at larger distance (m) |

Example – inside-bay with one command:

```bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py range_max:=0.5 z_max:=0.5 eps:=0.15 min_samples:=15
```

**Tracker tuning (fewer false tracks):** Increase `min_observations_to_add` (default 5) so an object must be seen more frames in the same area before it becomes a tracked buoy. Decrease `candidate_max_consecutive_misses` (default 4) to drop candidates faster if they disappear. Decrease `association_threshold` (default 0.5 m) to match only detections closer to existing tracks.

**Requirements:** `unitree_lidar_ros2` and `rviz2` must be installed (same workspace for the driver).

---

### Option 2: Run nodes separately (4 terminals)

Use **4 terminals**, all with the workspace sourced:  
`source /path/to/your/ros2_ws/install/setup.bash`

#### Terminal 1 – LiDAR driver

```bash
ros2 launch unitree_lidar_ros2 launch.py
```

You should see the driver running and publishing. Check:

```bash
ros2 topic hz /unilidar/cloud
```

#### Terminal 2 – Range filter

The range filter clips by height (z) and horizontal range, and can accumulate points.  
**No launch file is provided**; use `ros2 run` with parameters.

**Option A – Normal range (e.g. 30 m, good for open water)**

```bash
ros2 run pointcloud_filters lidar_range_filter --ros-args \
  -p input_topic:=/unilidar/cloud \
  -p output_topic:=/points_filtered \
  -p z_min:=0.0 \
  -p z_max:=10.0 \
  -p range_max:=30.0 \
  -p use_tf_transform:=False
```

**Option B – Inside-bay / short range (0.5 m, for close buoys)**

For **responsive markers**, use the default (no accumulation). Only enable accumulation when you need denser clouds and accept extra delay:

```bash
ros2 run pointcloud_filters lidar_range_filter --ros-args \
  -p input_topic:=/unilidar/cloud \
  -p output_topic:=/points_filtered \
  -p z_min:=0.0 \
  -p z_max:=0.5 \
  -p range_max:=0.5 \
  -p use_tf_transform:=False
```

To use accumulation (denser cloud, more delay), add: `-p enable_accumulation:=True -p accumulation_window:=1.0`

Check filtered output:

```bash
ros2 topic hz /points_filtered
```

#### Terminal 3 – Buoy detector (DBSCAN)

**Normal range (tune `eps` for buoy size, typically ~0.6–1.2 m):**

```bash
ros2 launch pointcloud_filters buoy_detector.launch.py
```

**Inside-bay (tighter clusters):**

```bash
ros2 run pointcloud_filters buoy_detector --ros-args \
  -p eps:=0.15 \
  -p min_samples:=15 \
  -p min_lateral_extent:=0.05 \
  -p max_lateral_extent:=0.4
```

Check detections:

```bash
ros2 topic echo /buoy_detections
```

#### Terminal 4 (optional) – Buoy tracker + RViz

For persistent IDs and visualization:

```bash
# Tracker (min_observations_for_publish:=1 or 2 makes new buoys appear sooner; 3 reduces flicker)
ros2 run pointcloud_filters buoy_tracker --ros-args \
  -p association_threshold:=0.3 \
  -p max_consecutive_misses:=5 \
  -p min_observations_for_publish:=2

# In another terminal: visualizer
ros2 run pointcloud_filters tracked_buoy_visualizer --ros-args \
  -p show_text_labels:=True
```

Then start RViz and add the topics below.

---

## 2. Check that output is correct

| What to check | Command / action |
|---------------|------------------|
| Raw LiDAR is publishing | `ros2 topic hz /unilidar/cloud` (e.g. ~10–20 Hz) |
| Filtered cloud is publishing | `ros2 topic hz /points_filtered` |
| Detections are published | `ros2 topic echo /buoy_detections` (see `range`, `bearing`, `confidence`) |
| Tracker (if used) | `ros2 topic echo /tracked_buoys` |

**RViz (recommended)**

1. Run: `rviz2`
2. Set **Fixed Frame** to: `unilidar_lidar`
3. Add by topic:
   - **PointCloud2**: `/unilidar/cloud` (raw) – point size e.g. `0.05`
   - **PointCloud2**: `/points_filtered` (filtered)
   - **MarkerArray**: `/tracked_buoy_markers` (if tracker + visualizer are running)

If you see the point cloud and markers moving with the LiDAR/buoys, the pipeline is working.

---

## 3. Typical issues

- **No `/unilidar/cloud`**  
  Start the Unitree LiDAR driver first and ensure the device is connected and the driver launch succeeds.

- **No `/points_filtered`**  
  Start the range filter after the driver. If the filter runs but you see no data, check `range_max` and `z_min`/`z_max` (e.g. 0.5 m range for inside-bay, 30 m for normal).

- **No or bad buoy detections**  
  - For **normal range**: tune `eps` in the buoy detector (e.g. 0.8 → 1.0 if buoys split; 0.6 if they merge).  
  - For **inside-bay**: use the inside-bay parameters above (smaller `eps`, higher `min_samples`).

- **Wrong workspace**  
  Always `source install/setup.bash` from the workspace that contains `pointcloud_filters` and `unitree_lidar_ros2`.

---

## 4. Low-latency tips

- **Range filter:** Default is no accumulation (`enable_accumulation:=False`) for responsive markers. Use accumulation only when you need a denser merged cloud and accept delay; if used, keep `accumulation_window` small (e.g. 0.1–0.2 s).
- **Tracker:** Nodes use QoS depth=1 so the latest frame is always processed. For new buoys to appear sooner, set `min_observations_for_publish:=1` or `2` (default 3 reduces flicker but adds delay).
- **RANSAC:** The buoy detector uses fewer RANSAC iterations by default (80). Disable with `ransac_enabled:=False` or reduce `ransac_iterations` further if latency is critical and water plane removal is not needed.

---

## 5. Pipeline summary

```
LiDAR driver (unitree_lidar_ros2)
    → /unilidar/cloud
    → lidar_range_filter
    → /points_filtered
    → buoy_detector
    → /buoy_detections
    → buoy_tracker
    → /tracked_buoys
    → tracked_buoy_visualizer
    → /tracked_buoy_markers (for RViz)
```

For more detail on nodes and parameters, see [mapping/README.md](README.md). Section 4 above gives low-latency tips.
