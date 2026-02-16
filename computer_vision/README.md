# Computer Vision ROS2 Pipeline

Multi-camera object detection with TensorRT-optimized YOLO on ROS2 Humble: 3 cameras → preprocessing → inference → combiner. For full architecture and design, see [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md).

---

## Quick start

```bash
cd ~/autonomy-ws-25-26/computer_vision
source /opt/ros/humble/setup.bash
colcon build --packages-select cv_ros_nodes
source install/setup.bash
```

Set 15 fps on cameras before launch:

```bash
./set_camera_fps.sh
```

(Uses Think USB paths; on other hardware edit the script or see [Camera configuration](#camera-configuration) for manual commands.)

```bash
ros2 launch cv_ros_nodes launch_cv.py
```

### Task-specific launch (RoboBoat 2026)

| Task | Launch command |
|------|----------------|
| **Tasks 1–3** (Evacuation Route, Debris Clearance, Emergency Sprint) | `ros2 launch cv_ros_nodes launch_cv.py enable_indicator_buoy:=true enable_task4:=false enable_number_detection:=false` |
| **Task 4** (Supply Drop) | `ros2 launch cv_ros_nodes launch_cv.py enable_task4:=true enable_indicator_buoy:=false enable_number_detection:=false` |
| **Task 5** (Navigate the Marina) | `ros2 launch cv_ros_nodes launch_cv.py enable_number_detection:=true enable_indicator_buoy:=false enable_task4:=false` |
| **Task 6** (Harbor Alert) | Use base pipeline or same as Task 3/5; CV only supports navigation (yellow buoy, dock). |

For Task 5, ensure the number detection TensorRT engine exists (e.g. build from `task_specific/Docking/number_detection/` with `build_number_engine.sh`). Override path if needed: `number_detection_engine:=/path/to/number_detection.engine`.

---

## RoboBoat competition: best for detections and accuracy

For the actual competition you want **maximum detections per second** and **best accuracy** so autonomy gets fresh, frequent updates. Here’s what actually matters and what to use.

**What the robot uses:** The combiner and any autonomy nodes only receive **`/combined/detection_info`** (and `..._with_distance`) when the inference node **runs inference** and publishes. So “detections per second” = how often you run inference. Display options (e.g. drawing boxes on every frame) do **not** change what the robot gets—only inference rate does.

| Goal | Setting | Why |
|------|--------|-----|
| **Most detections, best accuracy** | **`inference_interval:=1`** (default) | Run inference on **every** frame. No skipped frames, so you never miss a buoy that appeared only on a skip. Max detections/sec and lowest latency. |
| **Faster inference (higher FPS)** | **FP16 TensorRT engine** | Build the engine with `--fp16` (~2× faster, minimal accuracy loss). So with interval=1 you get more FPS (e.g. 4–6 instead of 2–3 on Jetson). See [model_training/TENSORRT.md](model_training/TENSORRT.md). |
| **Confidence** | **`conf_threshold:=0.25`** (default) | Good balance. Use **`0.2`** only if you need more marginal detections (e.g. distant buoys, glare) and accept more false positives. |
| **Real cameras at competition** | **`ros2 launch cv_ros_nodes launch_cv.py`** (+ task flags) | Main launch uses real cameras and does **not** pass `inference_interval`; the node default is **1**, so you already get inference every frame. Do **not** use `sim_image_rgb` (that’s for sim). |
| **Sim / testing** | **`launch_cv_sim.py`** with **`inference_interval:=1`** | Matches competition: every frame inferred. Use `inference_interval:=2` only for smoother display when debugging; that halves the detection updates the robot would see. |
| **Jetson GPU memory** | **`single_camera:=true`** (sim only) | If you hit OOM with sim + CV, run one camera so the pipeline is stable. On real hardware use as many cameras as the GPU can handle. |

**Summary:** For competition, use **`launch_cv.py`** (real cameras) with task flags as in the table above. Ensure **`model.engine`** is built with **FP16** so inference is as fast as possible. Keep **`inference_interval=1`** (default) so every frame is sent to the model and the combiner gets the maximum number of detection updates. `draw_stale_boxes` only affects the operator display; with interval=1 every frame is fresh anyway.

**Build FP16 engine (recommended before competition):**
```bash
cd ~/autonomy-ws-25-26/computer_vision
python model_training/export_onnx.py model_training/aqua_main.pt
/usr/src/tensorrt/bin/trtexec --onnx=model_training/aqua_main.onnx --saveEngine=cv_scripts/model.engine --fp16 --memPoolSize=workspace:4096 --skipInference
```

---

## Camera mapping

The launch file uses **persistent USB device paths** by default (Think). Same ROS namespace per camera regardless of boot order or plug order.

| Camera | USB Port | Persistent Device Path | ROS Namespace |
|--------|----------|-------------------------|---------------|
| Camera 0 | 1.2.2 | `/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0` | `/camera0` |
| Camera 1 | 1.2.3 | `/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0` | `/camera1` |
| Camera 2 | 1.2.4 | `/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0` | `/camera2` |

**Benefits:** Persistent mapping (port 1.2.2 → `/camera0`, etc.); no extra setup (kernel symlinks). On other hardware use `v4l2-ctl --list-devices` and override `camera_devices` or edit `set_camera_fps.sh`.

**Troubleshooting:** `ls -la /dev/v4l/by-path/` — list by-path symlinks. `v4l2-ctl --list-devices` — verify cameras. `udevadm info --query=all --name=/dev/video0 | grep ID_PATH` — see which USB port a device is on.

**Backward compatibility:** To use `/dev/video*` instead: `ros2 launch cv_ros_nodes launch_cv.py camera_devices:=/dev/video0,/dev/video2,/dev/video4`. Persistent paths are recommended for production.

**Simulation:** When running with Gazebo (sim bridges publishing `/camera0/image_raw` etc.), use **`ros2 launch cv_ros_nodes launch_cv_sim.py`** or **`launch_cv.py use_sim:=true`** so the pipeline does not start v4l2 camera nodes. See [simulations/README.md](../simulations/README.md) for CV + sim, detections in RViz, and troubleshooting.

---

## Pipeline architecture

```
Camera0: /camera0/image_raw → preprocessing0 → /camera0/image_preprocessed → inference0 → /camera0/detections, /camera0/detection_info
Camera1: /camera1/image_raw → preprocessing1 → /camera1/image_preprocessed → inference1 → /camera1/detections, /camera1/detection_info
Camera2: /camera2/image_raw → preprocessing2 → /camera2/image_preprocessed → inference2 → /camera2/detections, /camera2/detection_info
         │                                                                                        │
         ├── (optional) task4_supply_processor: /camera{N}/image_preprocessed + /camera{N}/detection_info
         │                                              → /camera{N}/task4_detections ─────────────┤
         ├── (optional) indicator_buoy_processor: /camera{N}/image_preprocessed                    │
         │                                              → /camera{N}/indicator_detections ─────────┤
         └───────────────────────────────────────────────────────────────────────────────────────┘
                                                                                            ↓
                                                                                    combiner → /combined/detection_info
```

Combiner subscribes to `/camera{N}/detection_info`, `/camera{N}/task4_detections` (when Task4 enabled), and `/camera{N}/indicator_detections` (when indicator buoy enabled). Use the launch file to start cameras, preprocessing, inference, and combiner; optionally task4 and indicator buoy via launch flags.

---

## Final combined CV output (downstream)

The **maritime distance estimator** is always included in the launch file. The **main final CV output** for mission/planning and fusion is:

| Topic | Type | Description |
|-------|------|-------------|
| **`/combined/detection_info_with_distance`** | `std_msgs/String` (JSON) | **Final CV output**: combined detections from all 3 cameras with `camera_id`, `class_name`, `score`, `bbox`, `bearing_deg`, `elevation_deg`, **`distance_m`**, **`distance_ft`**, plus `camera_info` and `camera_stats`. Use this for navigation and CV–LiDAR fusion. ~15 Hz. |

| Topic | Type | Description |
|-------|------|-------------|
| `/combined/detection_info` | `std_msgs/String` (JSON) | Same structure without distance (intermediate output; distance estimator consumes this and publishes the final topic above). |

**Sample output — `/combined/detection_info`** (excerpt; the payload is in the `data` field as JSON):

```json
{
  "timestamp": 1739123456.789,
  "num_cameras": 3,
  "num_active_cameras": 3,
  "total_detections": 2,
  "camera_info": {
    "frame_width": 1920,
    "frame_height": 1200,
    "horizontal_fov_deg": 85.0,
    "vertical_fov_deg": 69.0,
    "mounting_angles_deg": [-57.7, 0.0, 57.7]
  },
  "camera_stats": {
    "0": {"status": "active", "num_detections": 1, "fps": 14.2},
    "1": {"status": "active", "num_detections": 1, "fps": 14.5},
    "2": {"status": "active", "num_detections": 0, "fps": 14.1}
  },
  "detections": [
    {
      "camera_id": 0,
      "class_id": 3,
      "class_name": "red_buoy",
      "score": 0.87,
      "bbox": [420, 380, 580, 620],
      "bearing_deg": -32.5,
      "elevation_deg": 2.1
    },
    {
      "camera_id": 1,
      "class_id": 5,
      "class_name": "yellow_buoy",
      "score": 0.91,
      "bbox": [880, 450, 1020, 710],
      "bearing_deg": 8.2,
      "elevation_deg": -4.3
    }
  ]
}
```

**Sample output — `/combined/detection_info_with_distance`** (same structure with distance fields per detection):

```json
{
  "timestamp": 1739123456.789,
  "num_cameras": 3,
  "num_active_cameras": 3,
  "total_detections": 2,
  "camera_info": { "frame_width": 1920, "frame_height": 1200 },
  "detections": [
    {
      "camera_id": 0,
      "class_id": 3,
      "class_name": "red_buoy",
      "score": 0.87,
      "bbox": [420, 380, 580, 620],
      "bearing_deg": -32.5,
      "elevation_deg": 2.1,
      "distance_m": 4.52,
      "distance_ft": 14.8,
      "reference_height_m": 0.254
    },
    {
      "camera_id": 1,
      "class_id": 5,
      "class_name": "yellow_buoy",
      "score": 0.91,
      "bbox": [880, 450, 1020, 710],
      "bearing_deg": 8.2,
      "elevation_deg": -4.3,
      "distance_m": 6.11,
      "distance_ft": 20.0,
      "reference_height_m": 0.254
    }
  ],
  "distance_estimation": {
    "method": "specs_based",
    "camera_model": "AR0234",
    "distance_scale_factor": 0.87
  }
}
```

*(To view live: `ros2 topic echo /combined/detection_info --field data` or `ros2 topic echo /combined/detection_info_with_distance --field data` then parse the JSON.)*

---

## Launch overrides and optional nodes

**Overrides:** `resolution:=1920,1200`, `camera_devices:=/path1,/path2,/path3` (exactly 3), `engine_path:=/path/to/model.engine`, `conf_threshold:=0.25`, `staleness_threshold:=1.0`, `enable_task4:=true`, `enable_indicator_buoy:=true`, `enable_number_detection:=true`, `number_detection_engine:=/path/to/number_detection.engine`, `number_conf_threshold:=0.25`, `distance_scale_factor:=1.0` (one-point calibration, see below).

**Optional detection sources:**

- **Task4:** `enable_task4:=true` — Task4 detections in `/combined/detection_info` with `source: "task4"`, `class_name` `yellow_supply_drop` / `black_supply_drop`.
- **Indicator buoy (Tasks 1–3):** `enable_indicator_buoy:=true` — red/green colour indicator buoys in combined output as `red_indicator_buoy` / `green_indicator_buoy`.
- **Number detection (Task 5):** `enable_number_detection:=true` and valid `number_detection_engine` — digits 1–3 as `digit_1` / `digit_2` / `digit_3` in combined output.

Set 15 fps on devices before launch (run `./set_camera_fps.sh` or see [Camera configuration](#camera-configuration)).

**One-point calibration (distance):** The pipeline multiplies all distance estimates by a **distance scale factor** so they match real measurements. Use one or more photos of a buoy (or other object) at a **known distance** and **known size** to compute that factor; then pass it at launch (e.g. `distance_scale_factor:=0.77`). See **[camera_calibration/README.md](camera_calibration/README.md)** for what the scale factor means, one-photo steps, and **multiple photos at different distances** for a more accurate average.

---

## Camera configuration

**Resolution:** Default 1920×1200. Override: `resolution:=W,H` (e.g. `resolution:=1920,1200`).

**Devices:** Default = persistent USB paths for Think (see [Camera mapping](#camera-mapping)). On Jetson or other hardware, use `v4l2-ctl --list-devices` and pass `camera_devices:=/path1,/path2,/path3` (exactly 3).

**15 fps:** `v4l2_camera` has no FPS parameter. Before launch:

```bash
./set_camera_fps.sh
```

Or manually (Think paths; adjust on other hardware):

```bash
for d in /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0 /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0 /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0; do v4l2-ctl -d "$d" --set-parm 15; done
# Or current video numbers (may change after reboot): for d in /dev/video0 /dev/video2 /dev/video4; do v4l2-ctl -d "$d" --set-parm 15; done
```

**Checking cameras (e.g. on laptop):** With the camera(s) plugged in, run on the host:

```bash
v4l2-ctl --list-devices
ls -la /dev/v4l/by-path/
```

Use the by-path symlinks (or `/dev/video0`, etc.) as `camera_devices`. The launch file expects exactly three device paths; for one camera on a laptop, use that path for all three only if you need to test (some drivers allow it), or run a single camera node and one preprocessing/inference/combiner chain manually.

**Troubleshooting:** Verify FPS: `v4l2-ctl -d <device> --get-parm`. List devices: `v4l2-ctl --list-devices`. List formats: `v4l2-ctl -d <device> --list-formats-ext`.

---

## Manual node commands

When not using the launch file (e.g. debugging). Start cameras first, then preprocessing, then inference, then combiner.

**Camera nodes (v4l2_camera):**

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera1
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera2
# Or use /dev/video0, /dev/video2, /dev/video4 if overriding camera_devices
```

**CV nodes:** Preprocessing: `ros2 run cv_ros_nodes vision_preprocessing --camera_id 0` (and 1, 2). Inference: `ros2 run cv_ros_nodes vision_inference --camera_id 0 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine` (and 1, 2). Combiner: `ros2 run cv_ros_nodes vision_combiner` (optional: `--staleness_threshold 0.5`). Task4: `ros2 run cv_ros_nodes task4_supply_processor`. Indicator buoy: `ros2 run cv_ros_nodes indicator_buoy_processor`. Camera viewer (save images): `ros2 run cv_ros_nodes camera_viewer --camera_id 0 --output_dir ./camera_images`.

---

## Monitoring

Use `camera_info` for rate (image_raw is large). Example:

```bash
ros2 topic hz /camera0/camera_info --window 20
ros2 topic hz /combined/detection_info --window 20
ros2 topic echo /combined/detection_info --once --field data | jq .
ros2 topic list
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera{N}/image_raw` | `sensor_msgs/Image` | Raw camera feed (1920×1200) |
| `/camera{N}/image_preprocessed` | `sensor_msgs/Image` | Preprocessed image (pass-through) |
| `/camera{N}/detections` | `sensor_msgs/Image` | Image with bounding boxes drawn |
| `/camera{N}/detection_info` | `std_msgs/String` | JSON detection metadata; bbox in preprocessed frame |
| `/camera{N}/task4_detections` | `std_msgs/String` | Task4 supply-drop detections (when `enable_task4:=true`) |
| `/camera{N}/indicator_detections` | `std_msgs/String` | Indicator buoy detections (when `enable_indicator_buoy:=true`) |
| `/combined/detection_info` | `std_msgs/String` | Combined detections; `class_name`, `bearing_deg`, `elevation_deg`; `camera_info` |

---

## Detection info format

**Per-camera (`/camera{N}/detection_info`):** Bbox in preprocessed frame (e.g. 1920×1200). Top-level: `camera_id`, `timestamp`, `frame_width`, `frame_height`, `detections` array. Each detection: `class_id`, `score`, `bbox` `[x1,y1,x2,y2]`.

**Task4 (`/camera{N}/task4_detections`):** `detections` with `type` (`yellow_supply_drop` / `black_supply_drop`), `shape_bbox`, `vessel_bbox`, `source: "task4"`.

**Combined (`/combined/detection_info`):** Top-level: `timestamp`, `num_cameras`, `num_active_cameras`, `total_detections`, `camera_info`, `camera_stats` (per-camera status/fps), `detections`. Each detection: `camera_id`, `class_id`, `class_name`, `score`, `bbox` (local camera frame), `bearing_deg`, `elevation_deg`. Task4 entries include `source: "task4"`, `type`. Class names from `class_mapping.yaml` (YOLO 0–8, indicator 9–10, supply 11–12, numbers 20–22). Combiner uses latest value per camera and excludes cameras older than `staleness_threshold`.

Example combined detection entry:

```json
{
  "camera_id": 0,
  "class_id": 2,
  "class_name": "green_pole_buoy",
  "score": 0.85,
  "bbox": [100, 150, 200, 250],
  "bearing_deg": -45.2,
  "elevation_deg": 3.1
}
```

**Downstream (mission/control):** Subscribe to `/combined/detection_info`. Detections use global frame (3×frame_width×frame_height, e.g. 5760×1200); `bbox` is `[x1,y1,x2,y2]` in that frame; `global_frame` in payload. Task4 supply drops have `source == "task4"` and `type` in `"yellow_supply_drop"`, `"black_supply_drop"`.

---

## Platform

Development and testing are on a **Think** (x86). Default camera device paths in `set_camera_fps.sh` and the launch file are for this machine; on other hardware (e.g. Jetson) use `v4l2-ctl --list-devices` and set `camera_devices:=...` or edit the script. TensorRT engines can be built on the Think or on the deployment target (e.g. Jetson Orin).

---

## Prerequisites

- **OS:** Ubuntu 22.04  
- **ROS2:** Humble  
- **Python:** 3.10+  
- **GPU:** NVIDIA with CUDA  

**ROS2 packages:** `ros-humble-v4l2-camera`, `ros-humble-cv-bridge`  

**Python:** `numpy<2`, `opencv-python`, `tensorrt`, `pycuda`

---

## Project structure

```
computer_vision/
├── src/
│   ├── cv_ros_nodes/       # cv_ros_nodes package (nodes, launch_cv.py)
│   └── cv_lidar_fusion/    # cv_lidar_fusion package (vision_lidar_fusion → /fused_buoys)
├── cv_scripts/             # model.engine, class_mapping.yaml
├── model_training/         # weights.pt, weights.onnx, test_inference.py, TENSORRT.md
├── set_camera_fps.sh       # Set 15 FPS on all 3 cameras (Think USB paths)
├── README.md
├── DESIGN_STRATEGY.md
├── FUSION_NODE_GUIDE.md
├── TOPICS_LIDAR_AND_CV.md  # Every LiDAR and CV topic output (reference)
└── DISTANCE_ESTIMATOR_CHANGES.md
```

---

## Documentation

| Doc | Description |
|-----|-------------|
| [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md) | Full design and architecture |
| [simulations/README.md](../simulations/README.md) | Sim + CV, run commands, troubleshooting |
| [model_training/TENSORRT.md](model_training/TENSORRT.md) | TensorRT setup and model conversion |
| [FUSION_NODE_GUIDE.md](FUSION_NODE_GUIDE.md) | CV + LiDAR fusion |
| [TOPICS_LIDAR_AND_CV.md](TOPICS_LIDAR_AND_CV.md) | Every LiDAR and CV topic output (reference) |
| [DISTANCE_ESTIMATOR_CHANGES.md](DISTANCE_ESTIMATOR_CHANGES.md) | Distance estimation (maritime_distance_estimator) |

---

## License

MIT License. Contributors: Lorenzo DeMarni.

*Last updated: January 2025*
