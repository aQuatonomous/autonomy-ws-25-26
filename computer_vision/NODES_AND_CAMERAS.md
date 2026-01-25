# Nodes and Cameras

Single reference for running nodes, camera configuration, launch, monitoring, topics, and detection formats.

---

## Pipeline Architecture (3 Cameras)

```
Camera0: /camera0/image_raw → preprocessing0 → /camera0/image_preprocessed → inference0 → /camera0/detections, /camera0/detection_info
Camera1: /camera1/image_raw → preprocessing1 → /camera1/image_preprocessed → inference1 → /camera1/detections, /camera1/detection_info
Camera2: /camera2/image_raw → preprocessing2 → /camera2/image_preprocessed → inference2 → /camera2/detections, /camera2/detection_info
         │                                                                                        │
         └── (optional) task4_supply_processor: /camera{N}/image_preprocessed + /camera{N}/detection_info
                                                       → /camera{N}/task4_detections ─────────────┘
                                                                                            ↓
                                                                                    combiner → /combined/detection_info
```

Combiner subscribes to `/camera{N}/detection_info` and `/camera{N}/task4_detections` (when Task4 enabled). Start **camera nodes** first, then **preprocessing**, then **inference**, then **combiner**; optionally **task4_supply_processor** (launch with `enable_task4:=true`).

---

## Launch (all nodes)

```bash
ros2 launch cv_ros_nodes launch_cv.py
```

**Overrides:** `resolution:=1920,1200`, `camera_devices:=/dev/video0,/dev/video2,/dev/video4`, `engine_path:=/path/to/model.engine`, `conf_threshold:=0.25`, `staleness_threshold:=1.0`, `enable_task4:=true` (to run Task4 supply processor).

Set 15 fps on devices before launch (see [Camera configuration](#camera-configuration)).

---

## Camera configuration

### Resolution

- **Default:** 1920x1200  
- **Override:** `resolution:=W,H` (e.g. `resolution:=1920,1200`)

### Devices

- **Default:** `/dev/video0`, `/dev/video2`, `/dev/video4`  
- **Override:** `camera_devices:=/dev/video0,/dev/video2,/dev/video4` (exactly 3, comma-separated)

### 15 fps

`v4l2_camera` has no FPS parameter. Before starting the pipeline:

```bash
for d in /dev/video0 /dev/video2 /dev/video4; do v4l2-ctl -d "$d" --set-parm 15; done
```

Use your actual device list if you override `camera_devices`.

### Troubleshooting

**Programmed FPS:**

```bash
for d in /dev/video0 /dev/video2 /dev/video4; do echo "=== $d ==="; v4l2-ctl -d "$d" --get-parm; done
```

**List devices:**

```bash
v4l2-ctl --list-devices
```

**List formats for one device:**

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

---

## Camera nodes (manual)

```bash
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
ros2 run cv_ros_nodes vision_combiner --deduplicate_overlap
ros2 run cv_ros_nodes vision_combiner --deduplicate_overlap --overlap_zone_width 0.20 --overlap_y_tolerance 0.15
ros2 run cv_ros_nodes vision_combiner --use_timestamp_sync --sync_window 0.05
```

Subscribes: `/camera0/detection_info`, `/camera1/detection_info`, `/camera2/detection_info`; `/camera0/task4_detections`, `/camera1/task4_detections`, `/camera2/task4_detections`. Publishes: `/combined/detection_info`. All `bbox` in combined output are in the **global frame** (3×frame_width×frame_height, e.g. 5760×1200); payload includes `global_frame: { "width": W, "height": H }` from detection_info. Default: latest-value with 1.0s staleness.

### Task4 supply processor (optional)

```bash
ros2 run cv_ros_nodes task4_supply_processor
```

Subscribes: `/camera{N}/image_preprocessed`, `/camera{N}/detection_info` (N=0,1,2). Publishes: `/camera{N}/task4_detections` (JSON). Matches shape detections (class 6 cross, 8 triangle) to yellow/black blobs; `shape_bbox` and `vessel_bbox` are in the preprocessed frame (camera resolution, e.g. 1920×1200). Launched with `enable_task4:=true` via `launch_cv.py`.

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
| `/combined/detection_info` | `std_msgs/String` | Combined detections; `bbox` in global frame 3×frame_width×frame_height (e.g. 5760×1200); `global_frame` from detection_info |

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

All `detections[].bbox` are in the **global frame 1920×480** (`[x1,y1,x2,y2]`); x-offset = `camera_id * 640`. Payload includes `global_frame: { "width": 1920, "height": 480 }`. Task4 supply drops have `source == "task4"` and `type` in `"yellow_supply_drop"`, `"black_supply_drop"`.

```json
{
  "timestamp": 1234567890.123,
  "num_cameras": 3,
  "num_active_cameras": 2,
  "num_stale_cameras": 1,
  "num_no_data_cameras": 0,
  "staleness_threshold": 1.0,
  "total_detections": 5,
  "global_frame": { "width": 1920, "height": 480 },
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
      "score": 0.85,
      "bbox": [100, 150, 200, 250]
    },
    {
      "camera_id": 1,
      "type": "yellow_supply_drop",
      "class_id": 8,
      "score": 0.9,
      "source": "task4",
      "bbox": [800, 80, 880, 160]
    }
  ]
}
```

**Combiner behavior:** Latest value per camera; staleness filtering (cameras older than `staleness_threshold` excluded); status `active`, `stale`, `no_data`. Each detection includes `camera_id`. Frame dimensions from `frame_width`/`frame_height` in detection_info. Bbox from inference and Task4 are already in the preprocessed frame; combiner adds `camera_id * frame_width` to x for the global frame.
