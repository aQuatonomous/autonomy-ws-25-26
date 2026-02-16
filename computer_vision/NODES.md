# Computer Vision – Nodes and pipeline

Build, launch, manual node commands, and topic inputs/outputs. For “what to run at competition” see [COMPETITION.md](COMPETITION.md). For architecture and message formats see [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md).

---

## Build

```bash
cd ~/autonomy-ws-25-26/computer_vision
source /opt/ros/humble/setup.bash
colcon build --packages-select cv_ros_nodes
source install/setup.bash
```

Optional: `cv_lidar_fusion` for CV–LiDAR fusion (`/fused_buoys`). Build with `--packages-select cv_ros_nodes cv_lidar_fusion` if needed.

---

## Launch

- **Real cameras:** `ros2 launch cv_ros_nodes launch_cv.py` (set 15 fps first: `./set_camera_fps.sh`).
- **Simulation:** `ros2 launch cv_ros_nodes launch_cv_sim.py` or `ros2 launch cv_ros_nodes launch_cv.py use_sim:=true`.

Overrides: `resolution:=1920,1200`, `camera_devices:=/path1,/path2,/path3`, `engine_path:=/path/to/model.engine`, `conf_threshold:=0.25`, `enable_task4:=true`, `enable_indicator_buoy:=true`, `enable_number_detection:=true`, `number_detection_engine:=/path/to/number_detection.engine`, `distance_scale_factor:=1.0`.

---

## Pipeline (high level)

```
Camera0: /camera0/image_raw → preprocessing0 → /camera0/image_preprocessed → inference0 → /camera0/detections, /camera0/detection_info
Camera1: /camera1/image_raw → preprocessing1 → ... → inference1 → /camera1/detection_info
Camera2: /camera2/image_raw → preprocessing2 → ... → inference2 → /camera2/detection_info
         │
         ├── (optional) task4_supply_processor, indicator_buoy_processor
         └── combiner → /combined/detection_info → maritime_distance_estimator → /combined/detection_info_with_distance
```

---

## Manual node commands

When not using the launch file. Start cameras first, then preprocessing, then inference, then combiner.

**Camera nodes (v4l2_camera):**
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera0
# (and camera1, camera2 with respective devices)
```

**CV nodes:**
- Preprocessing: `ros2 run cv_ros_nodes vision_preprocessing --camera_id 0` (and 1, 2)
- Inference: `ros2 run cv_ros_nodes vision_inference --camera_id 0 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine` (and 1, 2)
- Combiner: `ros2 run cv_ros_nodes vision_combiner`
- Task4: `ros2 run cv_ros_nodes task4_supply_processor`
- Indicator buoy: `ros2 run cv_ros_nodes indicator_buoy_processor`
- Camera viewer (save images): `ros2 run cv_ros_nodes camera_viewer --camera_id 0 --output_dir ./camera_images`

---

## Topic inputs and outputs (per node)

| Node | Subscribes | Publishes |
|------|------------|-----------|
| v4l2_camera (per camera) | — | `/camera{N}/image_raw` |
| vision_preprocessing | `/camera{N}/image_raw` | `/camera{N}/image_preprocessed` |
| vision_inference | `/camera{N}/image_preprocessed` | `/camera{N}/detections`, `/camera{N}/detection_info` |
| task4_supply_processor | `/camera{N}/image_preprocessed`, `/camera{N}/detection_info` | `/camera{N}/task4_detections` |
| indicator_buoy_processor | `/camera{N}/image_preprocessed` | `/camera{N}/indicator_detections` |
| vision_combiner | `/camera{N}/detection_info`, (optional) task4, indicator | `/combined/detection_info` |
| maritime_distance_estimator | `/combined/detection_info` | `/combined/detection_info_with_distance` |

---

## Final CV output (downstream)

| Topic | Type | Description |
|-------|------|-------------|
| **`/combined/detection_info_with_distance`** | `std_msgs/String` (JSON) | **Final CV output**: combined detections with `camera_id`, `class_name`, `score`, `bbox`, `bearing_deg`, `elevation_deg`, `distance_m`, `distance_ft`. Use for navigation and CV–LiDAR fusion. ~15 Hz. |
| `/combined/detection_info` | `std_msgs/String` (JSON) | Same structure without distance (intermediate; distance estimator publishes the final topic above). |

---

## Monitoring

```bash
ros2 topic hz /camera0/camera_info --window 20
ros2 topic hz /combined/detection_info --window 20
ros2 topic echo /combined/detection_info --once --field data | jq .
ros2 topic list
```

For full topic list and message formats, see [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md).
