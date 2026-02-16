# Mapping – Competition and testing

What to run at competition, how to iterate (tuning, overrides), and troubleshooting. For pipeline and node details see [README.md](README.md). For WSL2 + USB LiDAR setup see [WSL2_LIDAR_SETUP.md](WSL2_LIDAR_SETUP.md).

---

## Quick run (one command)

From the **mapping** workspace:

```bash
cd ~/autonomy-ws-25-26/mapping
source install/setup.bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py
```

Starts LiDAR driver, range filter, buoy detector, tracker, visualizer, and RViz. Fixed frame: `unilidar_lidar`.

**No LiDAR hardware (e.g. WSL2 without USB):** run without the driver so the rest of the pipeline still works if `/unilidar/cloud` is published elsewhere:

```bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_lidar_driver:=false
```

**WSL2 + USB:** Attach the LiDAR with usbipd (see [WSL2_LIDAR_SETUP.md](WSL2_LIDAR_SETUP.md)), then use `launch_lidar_driver:=true` (default) or run the driver in a separate terminal with `lidar_port:=/dev/ttyACM0` if needed.

---

## Launch overrides

| Override | Example | Use case |
|----------|---------|----------|
| No LiDAR driver | `launch_lidar_driver:=false` | Driver runs elsewhere or no hardware |
| No RViz | `launch_rviz:=false` | Headless or your own RViz |
| Short range / inside-bay | `range_max:=0.5 z_max:=0.5` | Close buoys |
| Looser clustering (small buoys) | `eps:=0.9` or `eps:=1.0` | Points farther apart still form one cluster |
| Stricter tracking | `min_observations_to_add:=8 candidate_max_consecutive_misses:=3` | Fewer false tracks |
| Debug detector | `buoy_detector_log_level:=debug` | See why clusters are rejected |

**Inside-bay style (short range + tighter clusters):**

```bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py range_max:=0.5 z_max:=0.5 eps:=0.15 min_samples:=15
```

**Looser clustering (defaults already relaxed):**

```bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py eps:=0.9
ros2 launch pointcloud_filters buoy_pipeline.launch.py eps:=1.0
```

**Tighter clustering (if buoys merge):**

```bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py eps:=0.6
```

**Tracker:** Increase `min_observations_to_add` for fewer false buoys; decrease `candidate_max_consecutive_misses` to drop candidates faster; decrease `association_threshold` for stricter matching.

---

## Checking output

| Check | Command |
|-------|---------|
| Raw LiDAR frequency | `ros2 topic hz /unilidar/cloud --window 20` |
| Filtered cloud frequency | `ros2 topic hz /points_filtered --window 20` |
| Detections frequency | `ros2 topic hz /buoy_detections --window 20` |
| Tracked buoys frequency | `ros2 topic hz /tracked_buoys --window 20` |
| Fused buoys frequency | `ros2 topic hz /fused_buoys --window 20` |
| Detections (basic) | `ros2 topic echo /buoy_detections` |
| Tracked buoys (basic) | `ros2 topic echo /tracked_buoys` |
| Tracked buoys (full, no truncation) | `ros2 topic echo /tracked_buoys --no-arr` |
| Tracked buoys JSON (pretty) | `ros2 topic echo /tracked_buoys_json --once --field data \| jq .` |
| Fused buoys (full, no truncation) | `ros2 topic echo /fused_buoys --no-arr` |

**RViz:** Fixed frame `unilidar_lidar`. Add PointCloud2 `/unilidar/cloud`, `/points_filtered`; MarkerArray `/tracked_buoy_markers`. Labels show ID, distance (m), angle (°) with 0° = +X.

**Note:** Use `--no-arr` flag with `ros2 topic echo` to see full messages without truncation, especially useful for arrays with many buoys.

---

## Troubleshooting

- **No `/unilidar/cloud`** — Start the Unitree LiDAR driver; ensure device is connected (or use `launch_lidar_driver:=false` if cloud comes from elsewhere).
- **No `/points_filtered`** — Start the range filter after the driver. Check `range_max`, `z_min`, `z_max`.
- **No or bad buoy detections** — Tune `eps` (e.g. 0.8 → 1.0 if buoys split; 0.6 if they merge). Run with `buoy_detector_log_level:=debug` to see why clusters are rejected.
- **Wrong workspace** — Always `source install/setup.bash` from the **mapping** workspace (`~/autonomy-ws-25-26/mapping`).
- **CycloneDDS / rmw handle invalid** — Use FastRTPS: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` before launching.
- **ModuleNotFoundError: sklearn** — `sudo apt install python3-sklearn`.

For WSL2-specific issues (usbipd, serial port, RViz crashes), see [WSL2_LIDAR_SETUP.md](WSL2_LIDAR_SETUP.md).
