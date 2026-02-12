# Nodes and Cameras

Single reference for running nodes, camera configuration, launch, monitoring, topics, and detection formats.

---

## Pipeline Architecture (3 Cameras)

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

Combiner subscribes to `/camera{N}/detection_info`, `/camera{N}/task4_detections` (when Task4 enabled), and `/camera{N}/indicator_detections` (when indicator buoy enabled). Start **camera nodes** first, then **preprocessing**, then **inference**, then **combiner**; optionally **task4_supply_processor** (launch with `enable_task4:=true`) and **indicator_buoy_processor** (launch with `enable_indicator_buoy:=true`).

---

## Launch (all nodes)

```bash
ros2 launch cv_ros_nodes launch_cv.py
```

**Overrides:** `resolution:=1920,1200`, `camera_devices:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0`, `engine_path:=/path/to/model.engine`, `conf_threshold:=0.25`, `staleness_threshold:=1.0`, `enable_task4:=true` (Task4 supply processor), `enable_indicator_buoy:=true` (indicator buoy processor), `enable_number_detection:=true`, `number_detection_engine:=/path/to/number_detection.engine`, `number_conf_threshold:=0.25` (docking number detection).

**Optional detection sources (run or not):**

- **Task4:** `enable_task4:=true` runs the Task4 supply processor. When enabled, Task4 detections appear in `/combined/detection_info` with `source: "task4"`, `class_name` in `yellow_supply_drop` / `black_supply_drop`.
- **Indicator buoy (Tasks 1–3):** `enable_indicator_buoy:=true` runs the indicator buoy processor. Subscribes to `/camera{N}/image_preprocessed`; publishes `/camera{N}/indicator_detections`. When enabled, colour indicator buoy detections (red/green) appear in `/combined/detection_info` with `class_name` `red_indicator_buoy` or `green_indicator_buoy`, `indicator_color`, and whole-buoy bbox.
- **Number detection (docking, Task 5):** `enable_number_detection:=true` and a valid `number_detection_engine` run the number model inside the inference node. When enabled, number detections (class_id 20 = digit 1, 21 = digit 2, 22 = digit 3) appear with `class_name` `digit_1` / `digit_2` / `digit_3` in `/combined/detection_info`.

**Task presets (which optional nodes to run):**

- **Tasks 1–3:** `enable_indicator_buoy:=true`, `enable_task4:=false`, `enable_number_detection:=false` (main YOLO + indicator buoy on all 3 cameras).
- **Task 4:** `enable_task4:=true`, `enable_indicator_buoy:=false`, `enable_number_detection:=false` (supply vessel only).
- **Task 5:** `enable_number_detection:=true`, `enable_indicator_buoy:=false`, `enable_task4:=false` (number detection only).

Set 15 fps on devices before launch (see [Camera configuration](#camera-configuration)).

---

## Camera configuration

### Resolution

- **Default:** 1920x1200  
- **Override:** `resolution:=W,H` (e.g. `resolution:=1920,1200`)

### Devices

- **Default:** Persistent USB port paths (camera0: port 1.2.2, camera1: port 1.2.3, camera2: port 1.2.4)  
- **Override:** `camera_devices:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0` (exactly 3, comma-separated)

### 15 fps

`v4l2_camera` has no FPS parameter. Before starting the pipeline:

```bash
# Using persistent device paths
for d in /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0 /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0 /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0; do v4l2-ctl -d "$d" --set-parm 15; done

# Or using the current video device numbers (may change after reboot/replug)
for d in /dev/video0 /dev/video2 /dev/video4; do v4l2-ctl -d "$d" --set-parm 15; done
```

Use your actual device list if you override `camera_devices`.

### Troubleshooting

**Programmed FPS:**

```bash
# Using persistent device paths
for d in /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0 /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0 /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0; do echo "=== $d ==="; v4l2-ctl -d "$d" --get-parm; done

# Or using current video device numbers
for d in /dev/video0 /dev/video2 /dev/video4; do echo "=== $d ==="; v4l2-ctl -d "$d" --get-parm; done
```

**List devices:**

```bash
v4l2-ctl --list-devices
```

**List formats for one device:**

```bash
# Using persistent device path
v4l2-ctl -d /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0 --list-formats-ext

# Or using current video device number
v4l2-ctl -d /dev/video0 --list-formats-ext
```

---

## Camera nodes (manual)

```bash
# Using persistent USB port paths (recommended)
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera1
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0 -p image_size:="[1920,1200]" -r __ns:=/camera2

# Or using current video device numbers (may change after reboot/replug)
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -p image_size:="[1920,1200]" -r __ns:=/camera0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video2 -p image_size:="[1920,1200]" -r __ns:=/camera1
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video4 -p image_size:="[1920,1200]" -r __ns:=/camera2
```

When using the launch file, `resolution` and `camera_devices` overrides apply.

---

## CV nodes (per camera)

### Preprocessing

```bash
ros2 run cv_ros_nodes vision_preprocessing --camera_id 0
ros2 run cv_ros_nodes vision_preprocessing --camera_id 1
ros2 run cv_ros_nodes vision_preprocessing --camera_id 2
```

### Inference

```bash
ros2 run cv_ros_nodes vision_inference --camera_id 0 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine
ros2 run cv_ros_nodes vision_inference --camera_id 1 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine
ros2 run cv_ros_nodes vision_inference --camera_id 2 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine
```

### Combiner

```bash
ros2 run cv_ros_nodes vision_combiner
ros2 run cv_ros_nodes vision_combiner --staleness_threshold 0.5
ros2 run cv_ros_nodes vision_combiner --staleness_threshold 2.0
ros2 run cv_ros_nodes vision_combiner --use_timestamp_sync --sync_window 0.05
```

Subscribes: `/camera{N}/detection_info`, `/camera{N}/task4_detections`, `/camera{N}/indicator_detections`. **Merges** main (and number) detections from `detection_info` with Task4 detections and indicator buoy detections into a single list. Publishes: `/combined/detection_info`. Each detection has `bbox` in **local camera frame** (per-camera coordinates), `bearing_deg`/`bearing_rad` (boat-relative bearing, 0° = forward), `elevation_deg`/`elevation_rad`, and **`class_name`** (human-readable name from `class_mapping.yaml`, e.g. `red_pole_buoy`, `green_indicator_buoy`, `yellow_supply_drop`, `digit_2`). Payload includes `camera_info` (frame dimensions, FOV, mounting angles, intrinsics). Default: latest-value with 1.0s staleness.

### Task4 supply processor (optional)

```bash
ros2 run cv_ros_nodes task4_supply_processor
```

Subscribes: `/camera{N}/image_preprocessed`, `/camera{N}/detection_info` (N=0,1,2). Publishes: `/camera{N}/task4_detections` (JSON). Matches shape detections (class 6 cross, 8 triangle) to yellow/black blobs; `shape_bbox` and `vessel_bbox` are in the preprocessed frame (camera resolution, e.g. 1920×1200). Launched with `enable_task4:=true` via `launch_cv.py`.

### Indicator buoy processor (optional)

```bash
ros2 run cv_ros_nodes indicator_buoy_processor
ros2 run cv_ros_nodes indicator_buoy_processor --conf_threshold 0.3 --roi_conf_threshold 0.6
```

Subscribes: `/camera{N}/image_preprocessed` (N=0,1,2). Publishes: `/camera{N}/indicator_detections` (JSON). Runs the colour indicator buoy pipeline (diamonds + red/green indicator) on each preprocessed image. Detections include whole-buoy bbox (`shape_bbox`), `indicator_color` (red/green), and `class_id` 9 or 10. When enabled, combined output includes `class_name` `red_indicator_buoy` or `green_indicator_buoy`. Launched with `enable_indicator_buoy:=true` via `launch_cv.py`.

### Camera viewer (image saver)

```bash
ros2 run cv_ros_nodes camera_viewer --camera_id 0 --output_dir ./camera_images
```

Subscribes to `/camera{N}/image_raw`; saves `cam{N}.jpg`. Default output: `./camera_images/`.

---

## Monitoring

Use `camera_info` for rate; `image_raw` is too large for `ros2 topic hz`.

```bash
ros2 topic hz /camera0/camera_info --window 20
ros2 topic hz /camera1/camera_info --window 20
ros2 topic hz /camera2/camera_info --window 20

ros2 topic hz /camera0/image_preprocessed --window 20
ros2 topic hz /camera0/detections --window 20
ros2 topic hz /combined/detection_info --window 20

ros2 topic echo /camera0/detection_info
ros2 topic echo /camera0/task4_detections
ros2 topic echo /combined/detection_info

# View formatted JSON (single message)
ros2 topic echo /camera0/detection_info --once --field data | jq .
ros2 topic echo /combined/detection_info --once --field data | jq .
ros2 topic list
```

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera{N}/image_raw` | `sensor_msgs/Image` | Raw camera feed (1920x1200) |
| `/camera{N}/image_preprocessed` | `sensor_msgs/Image` | Preprocessed image (camera resolution, e.g. 1920×1200, pass-through) |
| `/camera{N}/detections` | `sensor_msgs/Image` | Image with bounding boxes drawn |
| `/camera{N}/detection_info` | `std_msgs/String` | JSON detection metadata; bbox in preprocessed frame; includes `frame_width`, `frame_height` |
| `/camera{N}/task4_detections` | `std_msgs/String` | Task4 supply-drop detections (shape_bbox, vessel_bbox in preprocessed frame); when `enable_task4:=true` |
| `/camera{N}/indicator_detections` | `std_msgs/String` | Indicator buoy detections (red/green); when `enable_indicator_buoy:=true` |
| `/combined/detection_info` | `std_msgs/String` | Combined detections; `bbox` in local camera frame; `class_name`, `bearing_deg`, `elevation_deg`; `camera_info` |

---

## Detection info format

### Per-camera (`/camera{N}/detection_info`)

Bbox is in the **preprocessed frame** (frame_width×frame_height, e.g. 1920×1200). Includes `frame_width` and `frame_height`. Example:

```json
{
  "camera_id": 0,
  "timestamp": 1234567890.123,
  "frame_id": "camera0",
  "num_detections": 2,
  "inference_time_ms": 12.5,
  "fps": 15.0,
  "frame_width": 1920,
  "frame_height": 1200,
  "detections": [
    {
      "class_id": 2,
      "score": 0.85,
      "x1": 100.0,
      "y1": 150.0,
      "x2": 200.0,
      "y2": 250.0,
      "width": 100.0,
      "height": 100.0,
      "bbox": [100, 150, 200, 250]
    }
  ]
}
```

### Per-camera Task4 (`/camera{N}/task4_detections`)

`shape_bbox` and `vessel_bbox` are in the **preprocessed frame** (same as image_preprocessed). Example:

```json
{
  "camera_id": 0,
  "timestamp": 1234567890.5,
  "detections": [
    {
      "type": "yellow_supply_drop",
      "class_id": 8,
      "score": 0.9,
      "shape_bbox": [120, 80, 200, 160],
      "vessel_bbox": [80, 200, 220, 320],
      "source": "task4"
    }
  ]
}
```

### Combined (`/combined/detection_info`)

Each detection has `bbox` in **local camera frame** (per-camera coordinates). Every detection includes **`class_name`** (e.g. `red_pole_buoy`, `green_indicator_buoy`, `yellow_supply_drop`, `digit_2`). Payload includes `camera_info` (frame dimensions, FOV, mounting angles, intrinsics). Each detection has `bearing_deg`/`bearing_rad` (boat-relative), `elevation_deg`/`elevation_rad`. Task4: `class_name` `yellow_supply_drop`/`black_supply_drop`. Indicator: `class_name` `red_indicator_buoy`/`green_indicator_buoy`.

```json
{
  "timestamp": 1234567890.123,
  "num_cameras": 3,
  "num_active_cameras": 2,
  "num_stale_cameras": 1,
  "num_no_data_cameras": 0,
  "staleness_threshold": 1.0,
  "total_detections": 5,
  "camera_info": { "frame_width": 1920, "frame_height": 1200, "horizontal_fov_deg": 85, "vertical_fov_deg": 69, "mounting_angles_deg": [-57.7, 0, 57.7], "fx": 1047.66, "fy": 873.0, "cx": 960, "cy": 600 },
  "camera_stats": {
    "0": {
      "status": "active",
      "num_detections": 2,
      "fps": 15.0,
      "timestamp": 1234567890.1,
      "time_since_update": 0.023
    },
    "1": {
      "status": "active",
      "num_detections": 2,
      "fps": 15.0,
      "timestamp": 1234567890.1,
      "time_since_update": 0.031
    },
    "2": {
      "status": "stale",
      "num_detections": 0,
      "time_since_update": 1.234,
      "staleness_threshold": 1.0,
      "fps": 0.0,
      "last_timestamp": 1234567888.9
    }
  },
  "detections": [
    {
      "camera_id": 0,
      "class_id": 2,
      "class_name": "green_pole_buoy",
      "score": 0.85,
      "bbox": [100, 150, 200, 250],
      "bearing_deg": -45.2,
      "elevation_deg": 3.1
    },
    {
      "camera_id": 1,
      "class_id": 8,
      "class_name": "yellow_supply_drop",
      "type": "yellow_supply_drop",
      "score": 0.9,
      "source": "task4",
      "bbox": [120, 80, 200, 160],
      "bearing_deg": -10.5,
      "elevation_deg": -2.0
    }
  ]
}
```

**Combiner behavior:** Latest value per camera; staleness filtering (cameras older than `staleness_threshold` excluded). Each detection has `camera_id`, `bbox` (local camera frame), `bearing_deg`/`bearing_rad`, `elevation_deg`/`elevation_rad`, and `class_name`. Class names come from `class_mapping.yaml` (YOLO 0–8, indicator 9–10, supply 11–12, numbers 20–22).

