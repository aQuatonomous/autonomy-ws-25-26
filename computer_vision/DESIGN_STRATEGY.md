# Computer Vision Pipeline - Design Strategy Document

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Node Architecture](#node-architecture)
4. [Topic Communication Matrix](#topic-communication-matrix)
5. [Message Formats](#message-formats)
6. [TensorRT Model Training Pipeline](#tensorrt-model-training-pipeline)
7. [Testing Procedures](#testing-procedures)
8. [Performance Targets](#performance-targets)
9. [Deployment Architecture](#deployment-architecture)
10. [Failure Modes and Mitigation](#failure-modes-and-mitigation)
11. [Configuration Parameters](#configuration-parameters)
12. [Task-Specific Computer Vision Requirements](#task-specific-computer-vision-requirements)
13. [Monitoring and Diagnostics](#monitoring-and-diagnostics)

**Related documentation:** [README.md](README.md) is the runbook (quick start, launch, cameras, nodes, topics). This document is the full design reference. For specific topics: [simulations/README.md](../simulations/README.md) (sim + CV), [DISTANCE_ESTIMATOR_CHANGES.md](DISTANCE_ESTIMATOR_CHANGES.md), [FUSION_NODE_GUIDE.md](FUSION_NODE_GUIDE.md), [model_training/TENSORRT.md](model_training/TENSORRT.md).

---

## Executive Summary

This document describes the complete design strategy for a multi-camera computer vision pipeline using ROS2 and TensorRT for real-time object detection. The system processes three synchronized camera feeds (video0, video2, video4) through independent preprocessing and inference pipelines, then combines detections into a unified output stream.

**Key Design Principles:**
- **Fault Isolation**: Independent pipelines per camera ensure single camera failures don't cascade
- **Scalability**: Modular design allows easy addition/removal of cameras
- **Performance**: TensorRT optimization for real-time inference (development on Think; deployment e.g. NVIDIA Jetson Orin)
- **Reliability**: Robust error handling and graceful degradation

---

## System Architecture

### High-Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Hardware Layer                                   │
├─────────────────────────────────────────────────────────────────────────┤
│  Camera 0 (/dev/video0)  │  Camera 1 (/dev/video2)  │  Camera 2 (/dev/video4) │
│  1920x1200 @ 15 FPS      │  1920x1200 @ 15 FPS      │  1920x1200 @ 15 FPS     │
└──────────────┬────────────┴──────────────┬────────────┴──────────────┬─────────┘
               │                           │                           │
┌──────────────▼────────────┐  ┌───────────▼───────────┐  ┌───────────▼───────────┐
│   Camera Driver Node 0    │  │  Camera Driver Node 1  │  │  Camera Driver Node 2  │
│  (v4l2_camera_node)       │  │  (v4l2_camera_node)    │  │  (v4l2_camera_node)   │
│  Namespace: /camera0      │  │  Namespace: /camera1   │  │  Namespace: /camera2  │
└──────────────┬────────────┘  └───────────┬───────────┘  └───────────┬───────────┘
               │                           │                           │
┌──────────────▼────────────┐  ┌───────────▼───────────┐  ┌───────────▼───────────┐
│  Preprocessing Node 0     │  │  Preprocessing Node 1  │  │  Preprocessing Node 2  │
│  (vision_preprocessing)   │  │  (vision_preprocessing)│  │  (vision_preprocessing)│
└──────────────┬────────────┘  └───────────┬───────────┘  └───────────┬───────────┘
               │                           │                           │
┌──────────────▼────────────┐  ┌───────────▼───────────┐  ┌───────────▼───────────┐
│   Inference Node 0        │  │   Inference Node 1     │  │   Inference Node 2    │
│  (vision_inference)      │  │  (vision_inference)   │  │  (vision_inference)   │
│  TensorRT Engine         │  │  TensorRT Engine       │  │  TensorRT Engine       │
└──────────────┬────────────┘  └───────────┬───────────┘  └───────────┬───────────┘
               │                           │                           │
               └───────────────┬───────────┴───────────────┬───────────┘
                               │                           │
                    ┌──────────▼───────────────────────────▼──────────┐
                    │      Detection Combiner Node                    │
                    │      (vision_combiner)                          │
                    │      - Aggregates all detections                │
                    │      - Health monitoring                        │
                    └───────────────────┬────────────────────────────┘
                                        │
                    ┌───────────────────▼────────────────────────────┐
                    │         Combined Detection Output                │
                    │         /combined/detection_info                │
                    └─────────────────────────────────────────────────┘
```

### Data Flow Pipeline

**Stage 1: Image Acquisition**
- Raw images captured at 1920x1200 resolution, 15 FPS (FPS set via v4l2-ctl; see [README.md](README.md))
- Published as ROS2 `sensor_msgs/Image` messages

**Stage 2: Preprocessing**
- Resize to 640x480 (configurable)
- Future: glare reduction, color correction, histogram equalization
- Output: Preprocessed images ready for inference

**Stage 3: Inference**
- TensorRT engine processes preprocessed images
- YOLO-based object detection
- Output: Bounding boxes, class IDs, confidence scores

**Stage 4: Aggregation**
- Combines detections from all cameras
- Output: Unified detection list with camera source tracking
- All detections from all cameras are preserved (no deduplication)

---

## Node Architecture

### 1. Camera Driver Nodes (v4l2_camera_node)

**Node Name**: `v4l2_camera_node` (3 instances with different namespaces)

**Purpose**: Interface with V4L2-compatible USB cameras

**Physical Configuration**:
- **Camera arrangement**: 3 cameras mounted side-by-side
- **Field of view per camera**: 85 degrees (horizontal)
- **Overlap between adjacent cameras**: 15 degrees
- **Total horizontal coverage**: ~245 degrees

**Instances**:
- `/camera0/v4l2_camera_node` (video0)
- `/camera1/v4l2_camera_node` (video2)
- `/camera2/v4l2_camera_node` (video4)

**Parameters**:
- `video_device`: `/dev/video0`, `/dev/video2`, `/dev/video4` (default; override via launch)
- `image_size`: `[1920, 1200]`
- `framerate`: v4l2_camera has no FPS parameter; set 15 fps via `v4l2-ctl --set-parm 15` before launch. See [README.md](README.md).
- `namespace`: `/camera0`, `/camera1`, `/camera2`

**Subscriptions**: None (hardware interface)

**Publications**:
- `/camera{N}/image_raw` (sensor_msgs/Image)
  - Encoding: `bgr8`
  - Resolution: 1920x1200
  - Frame rate: ~15 Hz (set via v4l2-ctl)
  - Header includes timestamp and frame_id

**Launch Command**: See [README.md](README.md) for full commands. Example: `ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -p image_size:="[1920,1200]" -r __ns:=/camera0`

---

### 2. Preprocessing Nodes (vision_preprocessing.py)

**Node Name**: `preprocess_camera{N}` (N = 0, 1, 2)

**Purpose**: Image preprocessing before inference (resize, normalization, enhancement)

**Class**: `PreprocessCamera`

**Parameters** (Command Line):
- `--camera_id`: Integer (0, 1, or 2) - Required

**Subscriptions**:
- `/camera{N}/image_raw` (sensor_msgs/Image)
  - Queue size: 10
  - Callback: `image_callback()`

**Publications**:
- `/camera{N}/image_preprocessed` (sensor_msgs/Image)
  - Encoding: `bgr8`
  - Resolution: camera resolution (e.g. 1920×1200, pass-through)
  - Frame rate: Matches input (up to 15 Hz)
  - Header: Preserved from input message

**Processing Steps**:
1. Convert ROS2 Image → OpenCV BGR format
2. Resize: 1920x1200 → 640x480
3. (Future: glare reduction, histogram equalization, color correction)
4. Convert OpenCV → ROS2 Image
5. Publish with preserved timestamp

**Performance Characteristics**:
- Processing time: ~2-5 ms per frame
- CPU-bound operation
- No GPU usage

**Launch Command**:
```bash
ros2 run cv_ros_nodes vision_preprocessing --camera_id 0
ros2 run cv_ros_nodes vision_preprocessing --camera_id 1
ros2 run cv_ros_nodes vision_preprocessing --camera_id 2
```

---

### 3. Inference Nodes (vision_inference.py)

**Node Name**: `inference_node_camera{N}` (N = 0, 1, 2)

**Purpose**: Run TensorRT-optimized YOLO inference on preprocessed images

**Class**: `InferenceNode`

**Dependencies**:
- TensorRT 10.3.0
- CUDA 12.6
- NVIDIA GPU (development: Think; deployment e.g. Jetson Orin)

**Parameters** (Command Line):
- `--camera_id`: Integer (0, 1, or 2) - Required
- `--engine_path`: String (default: `model.engine`) - Path to TensorRT engine file
- `--conf_threshold`: Float (default: 0.25) - Confidence threshold (0.0-1.0)

**Subscriptions**:
- `/camera{N}/image_preprocessed` (sensor_msgs/Image)
  - Queue size: 10
  - Callback: `image_callback()`

**Publications**:
- `/camera{N}/detections` (sensor_msgs/Image)
  - Encoding: `bgr8`
  - Resolution: preprocessed (camera resolution, e.g. 1920×1200)
  - Frame rate: Matches input
  - Content: Original image with bounding boxes drawn
  - Header: Preserved from input

- `/camera{N}/detection_info` (std_msgs/String)
  - Format: JSON
  - Frequency: Matches inference rate
  - Content: Detection metadata (see Message Formats section)

**Internal Components**:

**a) TensorRTInference Class**:
- `__init__()`: Loads TensorRT engine, allocates GPU memory
- `preprocess()`: Converts image to model input format (640x640, RGB, normalized)
- `infer()`: Executes inference on GPU
- `postprocess_yolo()`: Converts raw output to bounding boxes with NMS
- `draw_detections()`: Draws bounding boxes on image

**b) CudaRuntime Class**:
- Low-level CUDA memory management
- Handles GPU memory allocation/deallocation
- Manages host-device memory transfers

**Processing Pipeline**:
1. Receive preprocessed image (640x480 BGR)
2. Preprocess: Resize to 640x640, BGR→RGB, normalize [0,1], HWC→CHW
3. Copy to GPU memory
4. Execute TensorRT inference (async)
5. Copy results from GPU
6. Post-process: Extract boxes, apply confidence threshold, NMS
7. Draw detections on image
8. Publish detection image and JSON metadata

**Performance Characteristics**:
- Inference time: ~10-20 ms per frame (depends on model complexity)
- GPU-bound operation
- Memory: ~100-200 MB GPU memory per instance

**Launch Command**:
```bash
ros2 run cv_ros_nodes vision_inference --camera_id 0 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine --conf_threshold 0.25
ros2 run cv_ros_nodes vision_inference --camera_id 1 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine --conf_threshold 0.25
ros2 run cv_ros_nodes vision_inference --camera_id 2 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine --conf_threshold 0.25
```

---

### 4. Detection Combiner Node (vision_combiner.py)

**Node Name**: `detection_combiner`

**Purpose**: Aggregate detections from all cameras into unified output

**Class**: `DetectionCombiner`

**Parameters** (Command Line):
- `--staleness_threshold`: Float (default: 1.0) - Maximum age in seconds before excluding camera
- `--use_timestamp_sync`: Boolean flag - Enable timestamp-based synchronization (advanced)
- `--sync_window`: Float (default: 0.05) - Time window for timestamp matching (seconds)
- `--deduplicate_overlap`: Boolean flag - Enable overlap zone deduplication (default: disabled)
- `--overlap_zone_width`: Float (default: 0.15) - Fraction of frame width for overlap zone (0.15 = 15%)
- `--overlap_y_tolerance`: Float (default: 0.1) - Fraction of frame height for y-coordinate matching (0.1 = 10%)

**Subscriptions**:
- `/camera0/detection_info`, `/camera1/detection_info`, `/camera2/detection_info` (std_msgs/String) - Queue size: 10
- `/camera0/task4_detections`, `/camera1/task4_detections`, `/camera2/task4_detections` (std_msgs/String) - Queue size: 10

**Publications**:
- `/combined/detection_info` (std_msgs/String)
  - Format: JSON
  - Frequency: ~15 Hz (timer-based, also publishes on new data; matches camera rate)
  - Content: Combined detections with camera source tracking; all `bbox` in **global frame** (3×frame_width×frame_height, e.g. 5760×1200); `global_frame` from detection_info; Task4 supply drops have `source == "task4"` and `type` in `"yellow_supply_drop"`, `"black_supply_drop"`

**Functionality**:

**a) Detection Aggregation (Latest Value Approach)**:
- Maintains the most recent detection from each camera
- Combines all active detections into single list
- Simple and fast - no complex synchronization needed
- Works well when cameras have slightly different frame rates

**b) Staleness Filtering (Automatic Failure Detection)**:
- Tracks update timestamps for each camera
- Automatically excludes cameras whose latest detection is older than threshold
- Configurable staleness threshold (default: 1.0 seconds)
- Helps detect camera failures, processing stalls, or network issues
- Camera status: `active` (fresh data), `stale` (too old), `no_data` (never received)

**c) Overlap Zone Deduplication (Optional)**:
- **Default behavior**: All detections from all cameras are preserved - no deduplication
- **Optional feature**: Enable with `--deduplicate_overlap` flag
- For side-by-side camera geometry (85° FOV, 15° overlap):
  - Camera 0 right edge overlaps with Camera 1 left edge
  - Camera 1 right edge overlaps with Camera 2 left edge
- **Matching criteria** (when deduplication enabled):
  - Same `class_id`
  - Similar vertical position (y-coordinate within tolerance)
  - Both detections in overlap zones of their respective cameras
- **Selection**: Keeps detection with higher confidence score
- **Configuration**:
  - `--overlap_zone_width`: Fraction of frame width for overlap zone (default: 0.15 = 15%)
  - `--overlap_y_tolerance`: Fraction of frame height for y-coordinate matching (default: 0.1 = 10%)
- **When to use**: Enable if you want to reduce duplicate detections in overlap zones
- **When not to use**: Keep disabled if you want all detections for redundancy or if objects are very close together

**d) Health Monitoring**:
- Tracks camera status: `active`, `no_data`, `stale`
- Reports FPS and detection counts per camera
- Output includes counts: `num_active_cameras`, `num_stale_cameras`, `num_no_data_cameras`
- Logs warnings when cameras become stale

**e) Temporal Smoothing**:
- Timer-based publishing ensures output even if cameras are slow
- Prevents gaps in detection stream
- Publishes at ~15 Hz regardless of camera update rates

**Performance Characteristics**:
- Processing time: <1 ms per update
- CPU-bound operation
- Minimal memory footprint

**Launch Command**: See [README.md](README.md). Example: `ros2 run cv_ros_nodes vision_combiner` (default 1.0s staleness); `--staleness_threshold`, `--deduplicate_overlap`, `--use_timestamp_sync` and related args available.

**Staleness Filtering Example**:
- Camera0: Last update 0.02s ago → **INCLUDED** (active)
- Camera1: Last update 0.05s ago → **INCLUDED** (active)
- Camera2: Last update 1.5s ago → **EXCLUDED** (stale, threshold=1.0s)
- Output: Only detections from Camera0 and Camera1

**e) Task4 supply-drop merging**:
- Subscribes to `/camera{N}/task4_detections`; stores in `task4_detections` per camera
- Task4 entries: use `shape_bbox` (preprocessed frame) as `bbox`, `source: "task4"`, score from `score` or `confidence`
- Dedup (overlap) runs on local preprocessed frame; then each `bbox` is converted to global via `_to_global_bbox` (x += camera_id * frame_width)
- Same logic in `publish_combined` and `_publish_synchronized`

---

### 5. Task4 Supply Processor Node (task4_supply_processor.py)

**Node Name**: `task4_supply_processor`

**Purpose**: Match shape detections (class 6 cross, 8 triangle) from per-camera inference to yellow/black blobs via ROI-above-blob logic; publish supply-drop targets per camera for the combiner.

**Subscriptions**:
- `/camera{N}/image_preprocessed` (sensor_msgs/Image) – for blob detection (yellow, black)
- `/camera{N}/detection_info` (std_msgs/String) – shape detections; callback `_detection_callback(msg, camera_id)`

**Publications**:
- `/camera{N}/task4_detections` (std_msgs/String) – via `_task4_pubs[camera_id]`; JSON: `{ "camera_id": N, "timestamp": <float>, "detections": [ { "type": "yellow_supply_drop"|"black_supply_drop", "class_id": 6|8, "score": <float>, "shape_bbox": [x1,y1,x2,y2], "vessel_bbox": [x1,y1,x2,y2], "source": "task4" } ] }`; `shape_bbox` and `vessel_bbox` in **preprocessed frame** (camera resolution)

**Functionality**: Uses `camera_id` from the subscription; matches shapes to `_blobs_by_camera[camera_id]`; publishes only to `/camera{camera_id}/task4_detections`. No subscription to `/combined/detection_info`; no publisher to `/task4/detections`.

**Launch**: Optional; `launch_cv.py` with `enable_task4:=true`. Standalone: `ros2 run cv_ros_nodes task4_supply_processor`.

---

## Topic Communication Matrix

### Complete Topic List

| Topic Name | Message Type | Publisher Node | Subscriber Nodes | Frequency | Purpose |
|------------|--------------|----------------|------------------|-----------|---------|
| `/camera0/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera0) | preprocess_camera0 | ~15 Hz | Raw camera feed 0 |
| `/camera1/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera1) | preprocess_camera1 | ~15 Hz | Raw camera feed 1 |
| `/camera2/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera2) | preprocess_camera2 | ~15 Hz | Raw camera feed 2 |
| `/camera0/image_preprocessed` | sensor_msgs/Image | preprocess_camera0 | inference_node_camera0 | ~15 Hz | Preprocessed images 0 |
| `/camera1/image_preprocessed` | sensor_msgs/Image | preprocess_camera1 | inference_node_camera1 | ~15 Hz | Preprocessed images 1 |
| `/camera2/image_preprocessed` | sensor_msgs/Image | preprocess_camera2 | inference_node_camera2 | ~15 Hz | Preprocessed images 2 |
| `/camera0/detections` | sensor_msgs/Image | inference_node_camera0 | (monitoring/visualization) | ~15 Hz | Detection visualization 0 |
| `/camera1/detections` | sensor_msgs/Image | inference_node_camera1 | (monitoring/visualization) | ~15 Hz | Detection visualization 1 |
| `/camera2/detections` | sensor_msgs/Image | inference_node_camera2 | (monitoring/visualization) | ~15 Hz | Detection visualization 2 |
| `/camera0/detection_info` | std_msgs/String | inference_node_camera0 | detection_combiner, task4_supply_processor | ~15 Hz | Detection metadata 0 (bbox 640×640) |
| `/camera1/detection_info` | std_msgs/String | inference_node_camera1 | detection_combiner, task4_supply_processor | ~15 Hz | Detection metadata 1 (bbox 640×640) |
| `/camera2/detection_info` | std_msgs/String | inference_node_camera2 | detection_combiner, task4_supply_processor | ~15 Hz | Detection metadata 2 (bbox 640×640) |
| `/camera0/task4_detections` | std_msgs/String | task4_supply_processor | detection_combiner | on detection_info | Task4 supply drops, camera 0 (shape_bbox, vessel_bbox 640×480) |
| `/camera1/task4_detections` | std_msgs/String | task4_supply_processor | detection_combiner | on detection_info | Task4 supply drops, camera 1 |
| `/camera2/task4_detections` | std_msgs/String | task4_supply_processor | detection_combiner | on detection_info | Task4 supply drops, camera 2 |
| `/combined/detection_info` | std_msgs/String | detection_combiner | (downstream nodes) | ~15 Hz | Unified detections; bbox in global frame (3×frame_width×frame_height); global_frame from detection_info |

### Topic Dependency Graph

```
/camera0/image_raw
    ↓
/camera0/image_preprocessed
    ↓
/camera0/detections ──┐
/camera0/detection_info ──┐
                          │
/camera1/image_raw         │
    ↓                      │
/camera1/image_preprocessed│
    ↓                      │
/camera1/detections ──┐    │
/camera1/detection_info ──┤
                          ├──→ /combined/detection_info
/camera2/image_raw         │
    ↓                      │
/camera2/image_preprocessed│
    ↓                      │
/camera2/detections ──┐    │
/camera2/detection_info ───┘
```

---

## Message Formats

### 1. sensor_msgs/Image (Raw and Preprocessed)

**Standard ROS2 Image Message**:
- `header`: Standard ROS2 header (stamp, frame_id)
- `height`: Image height in pixels
- `width`: Image width in pixels
- `encoding`: `bgr8` (for all images)
- `is_bigendian`: Boolean
- `step`: Row step in bytes
- `data`: Raw pixel data (uint8 array)

**Example**:
- Raw: 1920x1200, ~7.2 MB per message
- Preprocessed: 640x480, ~0.9 MB per message

### 2. std_msgs/String (Detection Info)

**Individual Camera Detection Info** (`/camera{N}/detection_info`):

```json
{
  "camera_id": 0,
  "timestamp": 1234567890.123456,
  "frame_id": "camera0",
  "num_detections": 3,
  "inference_time_ms": 15.2,
  "fps": 32.5,
  "detections": [
    {
      "class_id": 2,
      "score": 0.85,
      "bbox": [100, 150, 200, 250]
    },
    {
      "class_id": 5,
      "score": 0.72,
      "bbox": [300, 400, 350, 450]
    },
    {
      "class_id": 2,
      "score": 0.68,
      "bbox": [500, 200, 600, 300]
    }
  ]
}
```

**Field Descriptions**:
- `camera_id`: Source camera identifier (0, 1, or 2)
- `timestamp`: ROS2 timestamp in seconds (float)
- `frame_id`: Frame identifier string
- `num_detections`: Number of detected objects
- `inference_time_ms`: Inference latency in milliseconds
- `fps`: Current processing rate (frames per second)
- `detections`: Array of detection objects
  - `class_id`: Object class identifier (integer)
  - `score`: Confidence score (0.0-1.0)
  - `bbox`: Bounding box `[x1, y1, x2, y2]` in **preprocessed frame** (frame_width×frame_height); `frame_width`, `frame_height` at top level

**Combined Detection Info** (`/combined/detection_info`):

All `detections[].bbox` are in the **global frame** (3×frame_width×frame_height, e.g. 5760×1200); x-offset = `camera_id * frame_width`. `global_frame` in payload (from detection_info). Task4 supply drops: `source == "task4"`, `type` in `"yellow_supply_drop"`, `"black_supply_drop"`.

```json
{
  "timestamp": 1234567890.123456,
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
      "num_detections": 3,
      "fps": 32.5,
      "timestamp": 1234567890.123,
      "time_since_update": 0.023
    },
    "1": {
      "status": "active",
      "num_detections": 2,
      "fps": 31.8,
      "timestamp": 1234567890.124,
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
      "class_id": 5,
      "score": 0.78,
      "bbox": [300, 400, 350, 450]
    }
  ],
  "synchronized": null
}
```

**Field Descriptions**:
- `timestamp`: Current system timestamp
- `num_cameras`: Total number of cameras (always 3)
- `num_active_cameras`: Number of cameras with fresh data (included in output)
- `num_stale_cameras`: Number of cameras with stale data (excluded from output)
- `num_no_data_cameras`: Number of cameras that never sent data
- `staleness_threshold`: Configured threshold for staleness detection (seconds)
- `total_detections`: Total number of detections from active cameras only
- `camera_stats`: Per-camera health and statistics
  - `status`: `"active"` (fresh data), `"no_data"` (never received), or `"stale"` (too old)
  - `num_detections`: Detections from this camera (0 if stale/no_data)
  - `fps`: Camera processing rate
  - `timestamp`: Last update timestamp (if active)
  - `time_since_update`: Seconds since last update
  - `staleness_threshold`: Threshold used (if stale)
  - `last_timestamp`: Last detection timestamp (if stale)
- `detections`: Combined array with `camera_id` and `bbox` in global frame (3×frame_width×frame_height); Task4 entries include `source: "task4"` and `type` (`"yellow_supply_drop"` or `"black_supply_drop"`)
  - **Note**: Only includes detections from `active` cameras (stale cameras excluded)
- `global_frame`: `{ "width": W, "height": H }` (e.g. 5760×1200, from detection_info)
- `synchronized`: `true` if timestamp sync used, `null` if latest value approach

---

## TensorRT Model Training Pipeline

### Overview

The model training pipeline converts a trained YOLO model (PyTorch/ONNX) into an optimized TensorRT engine for deployment (e.g. Jetson Orin; engines can also be built on the Think).

### Pipeline Stages

#### Stage 1: Model Training (External)

**Input**: Labeled dataset (images + annotations)
**Output**: Trained PyTorch model (`.pt` file)

**Tools**: PyTorch, YOLOv5/YOLOv8 training scripts
**Location**: External training environment (typically GPU workstation)

**Key Requirements**:
- Dataset format: COCO or YOLO format
- Model architecture: YOLOv5n, YOLOv5s, YOLOv8n, etc.
- Training parameters: Batch size, learning rate, epochs
- Validation split: 20% recommended

#### Stage 2: Model Export to ONNX

**Input**: Trained PyTorch model (`.pt`)
**Output**: ONNX model (`.onnx`)

**Process**: Use `torch.onnx.export` with fixed input size (e.g. 1×3×640×640) for TensorRT. For machine-specific steps and where files live, see [TENSORRT.md](TENSORRT.md).

**Validation**: Verify ONNX loads, test inference on samples, compare with PyTorch. **Tools**: PyTorch ONNX export, ONNX Runtime.

#### Stage 3: TensorRT Engine Conversion

**Input**: ONNX model (`.onnx`)
**Output**: TensorRT engine (`.engine`)

For machine-specific trtexec path, PATH, conversion commands (FP32/FP16/INT8), and Python API, see [TENSORRT.md](TENSORRT.md).

**Precision Selection Guide**:
- **FP32**: Highest accuracy, slowest (baseline)
- **FP16**: ~2x faster, <1% accuracy loss (recommended for Jetson / Think)
- **INT8**: ~4x faster, requires calibration dataset, 1-3% accuracy loss

**Optimization Parameters**:
- `workspace`: GPU memory for optimization (4096 MB recommended)
- `minShapes`: Minimum input dimensions
- `optShapes`: Optimal input dimensions (640x640)
- `maxShapes`: Maximum input dimensions

#### Stage 4: Engine Validation

**Input**: TensorRT engine (`.engine`)
**Output**: Validation report

**Process**: Use `model_training/test_inference.py` (or equivalent) on validation images; compare with ground truth, compute mAP/precision/recall. See [TENSORRT.md](TENSORRT.md).

**Metrics**:
- Mean Average Precision (mAP@0.5)
- Inference latency (ms)
- Throughput (FPS)
- Memory usage (GPU)

**Tools**: `test_inference.py`, custom validation scripts

#### Stage 5: Deployment

**Input**: Validated TensorRT engine
**Output**: Deployed model in ROS2 nodes

**Process**:
1. Copy `.engine` file to `cv_ros_nodes/` directory
2. Update `--engine_path` parameter in launch scripts
3. Verify inference nodes load engine successfully
4. Monitor performance in production

**File Locations**:
- Engine file: `/home/lorenzo/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine`
- ONNX file: `/home/lorenzo/autonomy-ws-25-26/computer_vision/model_training/weights.onnx`
- PyTorch weights: `/home/lorenzo/autonomy-ws-25-26/computer_vision/model_training/weights.pt`

### Model Versioning

**Naming Convention**: `model_v{version}_{precision}.engine`
- Example: `model_v1.0_fp16.engine`

**Version Tracking**:
- Maintain changelog with model versions
- Track performance metrics per version
- Document dataset and training parameters

---

## Testing Procedures

### Unit Testing

#### 1. Preprocessing Node Tests

**Test Cases**:
- **TC-PREP-001**: Verify pass-through (no resize; output size equals input)
- **TC-PREP-002**: Verify timestamp preservation
- **TC-PREP-003**: Verify encoding consistency (bgr8)
- **TC-PREP-004**: Test with missing/corrupted input
- **TC-PREP-005**: Measure processing latency (<5 ms)

**Test Script**: `test_preprocessing.py` (to be created)
```python
# Example test structure
def test_resize():
    # Create test image
    # Process through node
    # Verify output dimensions

def test_latency():
    # Measure processing time
    # Assert < 5 ms
```

#### 2. Inference Node Tests

**Test Cases**:
- **TC-INF-001**: Verify TensorRT engine loading
- **TC-INF-002**: Verify inference output format
- **TC-INF-003**: Test with empty/no detections
- **TC-INF-004**: Verify confidence threshold filtering
- **TC-INF-005**: Measure inference latency (<20 ms)
- **TC-INF-006**: Test GPU memory allocation/deallocation

**Test Script**: `test_inference.py` (exists)
```bash
# Single image test
python test_inference.py --image test.jpg

# Video test
python test_inference.py --video test.mp4

# Live camera test
python test_inference.py --camera 0
```

#### 3. Combiner Node Tests

**Test Cases**:
- **TC-COMB-001**: Verify detection aggregation from 3 cameras
- **TC-COMB-002**: Test with missing camera data
- **TC-COMB-003**: Test staleness detection
- **TC-COMB-004**: Verify all detections from all cameras are preserved
- **TC-COMB-005**: Test output format correctness

**Test Script**: `test_combiner.py` (to be created)

### Integration Testing

#### 1. End-to-End Pipeline Test

**Objective**: Verify complete pipeline from camera to combined output

**Procedure**:
1. Launch all camera nodes
2. Launch all preprocessing nodes
3. Launch all inference nodes
4. Launch combiner node
5. Monitor topics for 60 seconds
6. Verify:
   - All topics publishing at expected rates
   - Detection output contains valid data
   - No errors in node logs

**Success Criteria**:
- All nodes running without errors
- Topic frequencies within 10% of expected
- Combined detections include all camera sources

#### 2. Multi-Camera Synchronization Test

**Objective**: Verify cameras process independently without interference

**Procedure**:
1. Run pipeline with all 3 cameras
2. Monitor CPU/GPU usage per node
3. Verify no resource contention
4. Test camera failure scenarios

**Success Criteria**:
- Independent processing confirmed
- No performance degradation with multiple cameras
- Graceful handling of camera failures

### Performance Testing

#### 1. Latency Measurement

**Tools**: ROS2 topic timestamps, custom timing nodes

**Metrics**:
- End-to-end latency: Image capture → Combined output
- Per-stage latency: Preprocessing, inference, combination
- Network latency: ROS2 message transmission

**Targets**: See Performance Targets section

#### 2. Throughput Testing

**Procedure**:
- Run pipeline at maximum camera frame rate (15 FPS)
- Measure actual processing rates
- Identify bottlenecks

**Metrics**:
- FPS per camera
- Combined output rate
- GPU utilization
- CPU utilization

### Stress Testing

#### 1. High Load Test

**Procedure**:
- Run all nodes simultaneously
- Process at maximum frame rate
- Monitor for 10 minutes
- Check for memory leaks, crashes

#### 2. Failure Recovery Test

**Scenarios**:
- Camera disconnection
- GPU memory exhaustion
- Network congestion
- Node crashes

**Expected Behavior**:
- Graceful degradation
- Error logging
- Automatic recovery (where possible)

### Validation Testing

#### 1. Detection Accuracy Validation

**Procedure**:
- Run inference on labeled test set
- Compare detections with ground truth
- Calculate metrics: mAP, precision, recall

**Tools**: Custom validation scripts, COCO evaluation tools

#### 2. Real-World Testing

**Procedure**:
- Deploy in actual operating environment
- Collect detection data
- Analyze false positives/negatives
- Iterate on model improvements

---

## Performance Targets

### Latency Targets

| Stage | Target | Measurement Method |
|-------|--------|-------------------|
| Camera capture | <66.7 ms (15 FPS) | Hardware dependent |
| Preprocessing | <5 ms | Node internal timing |
| Inference | <20 ms | TensorRT timing |
| Combination | <1 ms | Node internal timing |
| **End-to-end** | **<50 ms** | Topic timestamp diff |

### Throughput Targets

| Metric | Target | Notes |
|--------|--------|-------|
| Per-camera FPS | ≥30 FPS | Sustained processing rate |
| Combined output rate | ≥30 Hz | Combiner node output |
| GPU utilization | 60-80% | Optimal range |
| CPU utilization | <80% | Per core |

### Resource Targets

| Resource | Target | Measurement |
|----------|--------|-------------|
| GPU memory | <2 GB | Total across all nodes |
| CPU memory | <1 GB | Per node |
| Network bandwidth | <100 MB/s | Total ROS2 traffic |

### Accuracy Targets

| Metric | Target | Notes |
|--------|--------|-------|
| mAP@0.5 | >0.75 | Mean Average Precision |
| Precision | >0.80 | False positive rate |
| Recall | >0.70 | False negative rate |
| FPS (inference) | >30 | Real-time requirement |

### Reliability Targets

| Metric | Target |
|--------|--------|
| Uptime | >99% |
| Mean Time Between Failures (MTBF) | >24 hours |
| Mean Time To Recovery (MTTR) | <5 minutes |

---

## Deployment Architecture

### Hardware Requirements

**Development platform**: Think (x86). Camera USB paths in `set_camera_fps.sh` and launch defaults are for this machine; on Jetson or other hardware, use `v4l2-ctl --list-devices` and override `camera_devices` or edit the script.

**Deployment target (e.g. boat)**: NVIDIA Jetson Orin
- GPU: NVIDIA Orin (2048 CUDA cores)
- CPU: ARM Cortex-A78AE (12 cores)
- Memory: 32 GB LPDDR5
- Storage: 64 GB+ NVMe SSD
- OS: Ubuntu 20.04 / JetPack 5.x

**Camera Requirements**:
- 3x USB 3.0 cameras
- Resolution: 1920x1200 minimum
- Frame rate: 15 FPS capable
- V4L2 compatible

**Network Requirements**:
- ROS2 DDS: Local network (no external dependency)
- Bandwidth: <100 MB/s for all topics

### Software Stack

**Operating System**:
- Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- Linux kernel 5.15+ (Tegra)

**ROS2 Distribution**:
- ROS2 Humble Hawksbill or ROS2 Foxy Fitzroy
- DDS Implementation: FastDDS or CycloneDDS

**Python Environment**:
- Python 3.8+
- Required packages:
  - `rclpy` (ROS2 Python client)
  - `sensor_msgs` (ROS2 message types)
  - `cv_bridge` (OpenCV-ROS2 bridge)
  - `opencv-python` (4.5+)
  - `numpy` (1.19+)
  - `tensorrt` (10.3.0)

**CUDA/TensorRT**:
- CUDA 12.6
- TensorRT 10.3.0
- cuDNN 8.x

**System Packages**:
- `v4l2-utils` (camera utilities)
- `ros-humble-v4l2-camera` (camera driver)

### Deployment Structure

```
/home/lorenzo/autonomy-ws-25-26/computer_vision/
├── src/cv_ros_nodes/
│   ├── cv_ros_nodes/
│   │   ├── vision_preprocessing.py
│   │   ├── vision_inference.py
│   │   ├── vision_combiner.py
│   │   ├── camera_viewer.py
│   │   └── launch/
│   │       └── launch_cv.py
│   ├── package.xml
│   └── setup.py
├── cv_scripts/
│   ├── model.engine
│   └── class_mapping.yaml
├── model_training/
│   ├── weights.pt
│   ├── weights.onnx
│   └── TENSORRT.md
├── set_camera_fps.sh
├── README.md
├── DESIGN_STRATEGY.md
├── FUSION_NODE_GUIDE.md
└── DISTANCE_ESTIMATOR_CHANGES.md
```

### Launch Configuration

**Launch file**: `launch_cv.py` (in `src/cv_ros_nodes/cv_ros_nodes/launch/`). Starts 3 v4l2 cameras, 3 preprocessing, 3 inference, 1 combiner. Set 15 fps on devices via v4l2-ctl before launch. See [README.md](README.md) for full commands and overrides.

**Launch Command**:
```bash
cd ~/autonomy-ws-25-26/computer_vision
source install/setup.bash
ros2 launch cv_ros_nodes launch_cv.py
```

### Process Management

**Recommended**: Use systemd or supervisor for production deployment

**systemd Service Example** (`vision-pipeline.service`):
```ini
[Unit]
Description=ROS2 Vision Pipeline
After=network.target

[Service]
Type=simple
User=lorenzo
WorkingDirectory=/home/lorenzo/autonomy-ws-25-26
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/lorenzo/autonomy-ws-25-26/computer_vision/install/setup.bash && ros2 launch cv_ros_nodes launch_cv.py'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Resource Management

**CPU Affinity**: Pin nodes to specific CPU cores
```bash
taskset -c 0-3 python3 vision_preprocessing.py --camera_id 0
taskset -c 4-7 python3 vision_inference.py --camera_id 0
```

**GPU Memory**: Monitor and limit per-process
```bash
# Set GPU memory fraction
export CUDA_VISIBLE_DEVICES=0
```

**Priority**: Set process priorities
```bash
nice -n -10 python3 vision_inference.py --camera_id 0
```

---

## Failure Modes and Mitigation

### Camera Failures

#### Failure Mode: Camera Disconnection

**Symptoms**:
- No messages on `/camera{N}/image_raw`
- Camera node logs show "Device not found"
- Preprocessing node reports no data

**Detection**:
- Monitor topic frequency: `ros2 topic hz /camera{N}/image_raw`
- Check camera node logs
- Combiner node reports `status: "no_data"` for affected camera

**Mitigation**:
- **Automatic**: Preprocessing node continues (no crash)
- **Automatic**: Inference node waits for data (no crash)
- **Automatic**: Combiner node excludes stale camera from output
- **Manual**: Restart camera node or reconnect hardware

**Recovery**:
- Reconnect camera hardware
- Restart camera node: `ros2 run v4l2_camera v4l2_camera_node ...`
- System automatically resumes processing

#### Failure Mode: Camera Frame Rate Degradation

**Symptoms**:
- Topic frequency drops below 30 Hz
- Increased latency in detection pipeline

**Detection**:
- Monitor: `ros2 topic hz /camera{N}/image_raw`
- Alert if frequency < 30 Hz for >5 seconds

**Mitigation**:
- Check USB bandwidth (use USB 3.0 ports)
- Reduce camera resolution if necessary
- Check for USB hub issues

### GPU Failures

#### Failure Mode: GPU Out of Memory

**Symptoms**:
- Inference node crashes with CUDA error
- Error: "out of memory" in logs
- GPU memory usage at 100%

**Detection**:
- Monitor GPU memory: `nvidia-smi`
- Check inference node logs for CUDA errors

**Mitigation**:
- **Prevention**: Limit concurrent inference instances
- **Prevention**: Use FP16 precision to reduce memory
- **Recovery**: Restart inference node(s)
- **Recovery**: Reduce batch size or model complexity

**Recovery**:
- Kill and restart affected inference nodes
- System continues with remaining cameras
- Gradually restart failed nodes

#### Failure Mode: GPU Overheating

**Symptoms**:
- GPU temperature >85°C
- Thermal throttling (reduced performance)
- Inference latency increases

**Detection**:
- Monitor: `nvidia-smi -q -d TEMPERATURE`
- Alert if temperature >80°C

**Mitigation**:
- **Immediate**: Reduce processing load
- **Immediate**: Enable GPU power limits
- **Long-term**: Improve cooling, check thermal paste

### Node Failures

#### Failure Mode: Preprocessing Node Crash

**Symptoms**:
- No messages on `/camera{N}/image_preprocessed`
- Node process not running
- Inference node reports no data

**Detection**:
- Monitor topic: `ros2 topic hz /camera{N}/image_preprocessed`
- Check node status: `ros2 node list`

**Mitigation**:
- **Automatic**: Process manager (systemd) restarts node
- **Automatic**: Inference node waits gracefully
- **Manual**: Restart node manually

**Recovery**:
- Process manager auto-restart (if configured)
- Manual restart: `python3 vision_preprocessing.py --camera_id N`

#### Failure Mode: Inference Node Crash

**Symptoms**:
- No messages on `/camera{N}/detections` or `/camera{N}/detection_info`
- Node process not running
- GPU memory not released

**Detection**:
- Monitor topics for missing data
- Check node status
- Monitor GPU memory (should decrease)

**Mitigation**:
- **Automatic**: Process manager restarts node
- **Automatic**: GPU memory cleanup on restart
- **Automatic**: Combiner node handles missing camera gracefully

**Recovery**:
- Process manager auto-restart
- Manual restart: `python3 vision_inference.py --camera_id N --engine_path model.engine`
- Verify GPU memory released: `nvidia-smi`

#### Failure Mode: Combiner Node Crash

**Symptoms**:
- No messages on `/combined/detection_info`
- Downstream nodes receive no data

**Detection**:
- Monitor topic: `ros2 topic hz /combined/detection_info`
- Check node status

**Mitigation**:
- **Critical**: This is a single point of failure
- **Prevention**: Robust error handling in combiner
- **Recovery**: Fast restart (<5 seconds)

**Recovery**:
- Process manager auto-restart
- Manual restart: `python3 vision_combiner.py`
- System resumes immediately (no data loss from cameras)

### Network/ROS2 Failures

#### Failure Mode: DDS Discovery Issues

**Symptoms**:
- Nodes cannot see each other's topics
- "No subscribers" warnings
- Topics exist but no data flow

**Detection**:
- Check: `ros2 topic list` (should show all topics)
- Check: `ros2 node list` (should show all nodes)
- Check DDS configuration

**Mitigation**:
- **Prevention**: Use same DDS implementation (FastDDS or CycloneDDS)
- **Prevention**: Configure DDS domain ID consistently
- **Recovery**: Restart ROS2 daemon: `ros2 daemon stop && ros2 daemon start`

#### Failure Mode: Topic Buffer Overflow

**Symptoms**:
- Dropped messages in logs
- Stale data in topics
- Increased latency

**Detection**:
- Monitor topic queue sizes
- Check for "dropped message" warnings

**Mitigation**:
- **Prevention**: Increase queue sizes (currently 10)
- **Prevention**: Ensure subscribers process fast enough
- **Recovery**: Restart slow nodes

### Model/Inference Failures

#### Failure Mode: Invalid TensorRT Engine

**Symptoms**:
- Inference node fails to start
- Error: "Failed to load engine"
- Engine file corrupted or incompatible

**Detection**:
- Check node startup logs
- Verify engine file exists and is readable

**Mitigation**:
- **Prevention**: Validate engine before deployment
- **Prevention**: Version control for engine files
- **Recovery**: Rebuild engine from ONNX
- **Recovery**: Use backup engine version

#### Failure Mode: Inference Accuracy Degradation

**Symptoms**:
- Increased false positives/negatives
- Detection quality decreases
- Model performance below targets

**Detection**:
- Monitor detection confidence scores
- Compare with validation metrics
- User feedback

**Mitigation**:
- **Prevention**: Regular model validation
- **Prevention**: Monitor detection statistics
- **Recovery**: Retrain model with updated dataset
- **Recovery**: Deploy new engine version

### System-Level Failures

#### Failure Mode: System Reboot

**Symptoms**:
- All nodes stop
- System restarts

**Mitigation**:
- **Automatic**: Process manager (systemd) auto-starts nodes on boot
- **Automatic**: Launch file configured for auto-start
- **Recovery**: System resumes within 30-60 seconds

#### Failure Mode: Disk Space Exhaustion

**Symptoms**:
- Log files fill disk
- Nodes cannot write logs
- System becomes unresponsive

**Detection**:
- Monitor disk usage: `df -h`
- Alert if usage >80%

**Mitigation**:
- **Prevention**: Log rotation configured
- **Prevention**: Limit log file sizes
- **Recovery**: Clean old logs, increase disk space

### Failure Mode Summary Table

| Failure Mode | Detection Method | Automatic Mitigation | Manual Recovery | Impact Severity |
|--------------|------------------|---------------------|-----------------|-----------------|
| Camera disconnect | Topic monitoring | Graceful degradation | Reconnect hardware | Medium |
| GPU OOM | nvidia-smi, logs | Node restart | Reduce load | High |
| Node crash | Process monitoring | Auto-restart | Manual restart | Medium |
| Combiner crash | Topic monitoring | Auto-restart | Manual restart | High |
| DDS failure | Topic/node list | Restart daemon | Fix config | High |
| Invalid engine | Startup logs | Use backup | Rebuild engine | High |
| System reboot | System logs | Auto-start on boot | Wait for recovery | Low |

---

## Configuration Parameters

### Camera Configuration

**File**: `config/camera_config.yaml`

```yaml
cameras:
  camera0:
    device: "/dev/video0"
    resolution: [1920, 1200]
    framerate: 60.0
    namespace: "/camera0"
  
  camera1:
    device: "/dev/video1"
    resolution: [1920, 1200]
    framerate: 60.0
    namespace: "/camera1"
  
  camera2:
    device: "/dev/video2"
    resolution: [1920, 1200]
    framerate: 60.0
    namespace: "/camera2"
```

### Preprocessing Configuration

**File**: `config/preprocessing_config.yaml`

```yaml
preprocessing:
  output_resolution: [1920, 1200]  # Pass-through at camera resolution
  enable_glare_reduction: false
  enable_histogram_equalization: false
  enable_color_correction: false
  queue_size: 10
```

### Inference Configuration

**File**: `config/inference_config.yaml`

```yaml
inference:
  engine_path: "model.engine"
  confidence_threshold: 0.25
  iou_threshold: 0.45
  input_size: [640, 640]
  precision: "fp16"  # fp32, fp16, int8
  queue_size: 10
```

### Combiner Configuration

**File**: `config/combiner_config.yaml`

```yaml
combiner:
  staleness_threshold: 1.0  # seconds
  publish_rate: 30.0  # Hz
  queue_size: 10
  deduplicate_overlap: false  # Enable to remove duplicates in 15° overlap zones
  overlap_zone_width: 0.15  # Fraction of frame width (15% = right/left edge)
  overlap_y_tolerance: 0.1  # Fraction of frame height for y-coordinate matching (10%)
  # Note: When deduplicate_overlap is false, all detections from all cameras are preserved
  # When enabled, detections in overlap zones are deduplicated based on class_id and position
```

### Performance Configuration

**File**: `config/performance_config.yaml`

```yaml
performance:
  target_fps: 30
  max_latency_ms: 50
  gpu_memory_limit_mb: 2048
  cpu_affinity:
    preprocessing: [0, 1, 2, 3]
    inference: [4, 5, 6, 7]
    combiner: [8, 9]
```

### Class Mapping Configuration

**File**: `config/class_mapping.yaml`

Maps class IDs (as output by the YOLO model) to human-readable object names.

```yaml
classes:
  0: black_buoy
  1: green_buoy
  2: green_pole_buoy
  3: red_buoy
  4: red_pole_buoy
  5: yellow_buoy
  6: cross
  7: dock
  8: triangle
```

**Usage**: The `class_utils.py` module provides utilities to convert between class IDs and names:
```python
from class_utils import class_id_to_name, class_name_to_id, is_buoy_class

# Convert class ID to name
name = class_id_to_name(3)  # Returns "red_buoy"

# Convert name to ID
class_id = class_name_to_id("red_buoy")  # Returns 3

# Check if class is a buoy
is_buoy = is_buoy_class(3)  # Returns True
```

---

## Task-Specific Computer Vision Requirements

### Overview

The computer vision pipeline provides the foundational perception capabilities required for all RoboBoat 2025 autonomy tasks. This section details how the vision system supports each task, what objects must be detected, and the specific computer vision capabilities required.

**Key Vision Capabilities Across All Tasks:**
- **Buoy Detection**: Red, green, black, and yellow buoys of various sizes
- **Color Indicator Detection**: Red/green color indicators mounted on custom buoys
- **Vessel Detection**: Yellow and black stationary vessels
- **Shape Detection**: Triangles, crosses/plus signs, and dock numbers
- **Spatial Understanding**: Bounding box coordinates for navigation and targeting

**Integration with Task Execution:**
- Vision detections are published to `/combined/detection_info`; `bbox` is in global 1920×480; `global_frame: { "width": 1920, "height": 480 }`
- Task4 supply drops: `source == "task4"`, `type` in `"yellow_supply_drop"`, `"black_supply_drop"`; also available on `/camera{N}/task4_detections` with `shape_bbox` and `vessel_bbox` in 640×480
- Task-specific nodes subscribe to detections and execute task logic
- Color indicator state detection is handled by a separate module (see `task_vision_processor.py` documentation)
- Real-time detection enables dynamic navigation and decision-making

---

### Task 1: Evacuation Route & Return

**Objective**: Navigate through entry and exit gates marked by pairs of red and green buoys.

**Computer Vision Requirements:**

**Primary Detections:**
- **Red Buoys** (class_id: 3): Port marker buoys (Taylor Made Sur-Mark, 39in height, 18in diameter)
- **Green Buoys** (class_id: 1): Starboard marker buoys (Taylor Made Sur-Mark, 39in height, 18in diameter)

**Vision Processing:**
1. **Gate Detection**: Identify pairs of red and green buoys forming navigation gates
   - Minimum detection range: 6 ft before first gate (autonomous navigation start point)
   - Must detect both buoys in a pair to identify gate center
   - Calculate gate centerline for navigation path planning

2. **Spatial Analysis**:
   - Estimate distance to buoys using bounding box size and camera calibration
   - Calculate relative position (port/starboard) for gate alignment
   - Monitor ASV position relative to gate centerline

3. **Safety Monitoring**:
   - Continuous detection of all gate buoys during transit
   - Alert if ASV trajectory would cause collision
   - Verify complete passage through gate (both buoys behind ASV)

**Output Requirements:**
- Real-time detection of red/green gate buoys
- Bounding box coordinates for navigation planning
- Gate centerline calculation
- Completion verification (entry and exit gates)

**Performance Targets:**
- Detection range: >20 meters
- Update rate: ≥30 Hz for real-time navigation
- False positive rate: <5% (critical for collision avoidance)

---

### Task 2: Debris Clearance

**Objective**: Navigate through channel, enter debris field, identify hazards/survivors, and return.

**Computer Vision Requirements:**

**Primary Detections:**
- **Gate Buoys**: Red (class_id: 3) and green (class_id: 1) buoys marking channel entrance/exit
- **Black Buoys** (class_id: 0): Obstacle buoys representing debris (Polyform A-0, 0.5ft height)
- **Color Indicator Buoys**: Red or green indicators on custom buoys
  - Red indicator = hazard to avoid and report
  - Green indicator = survivor to rescue, circle, and report

**Vision Processing:**

1. **Channel Navigation**:
   - Detect red/green gate buoys forming channel boundaries
   - Maintain position within channel (between gate pairs)
   - Avoid contact with channel markers

2. **Debris Field Scanning**:
   - Detect all black obstacle buoys (debris)
   - Detect color indicator buoys and classify color (red/green)
   - Estimate positions of all detected objects for reporting

3. **Color Indicator Classification**:
   - Detect custom buoy with color indicator
   - Classify indicator color (red vs green) - see Color Indicator Detection section
   - Determine if indicator requires action (green = circle, red = avoid)

4. **Spatial Mapping**:
   - Build map of debris field with all object positions
   - Calculate relative positions for navigation planning
   - Identify safe paths through debris field

**Output Requirements:**
- Detection of all gate buoys (red/green)
- Detection of all black obstacle buoys
- Detection and color classification of color indicator buoys
- Position estimates (lat/long) for all detected objects
- Real-time obstacle map for path planning

**Performance Targets:**
- Detection range: >30 meters for debris field scanning
- Color classification accuracy: >95% (critical for survivor identification)
- Position estimation accuracy: ±1 meter (for reporting requirements)

---

### Task 3: Emergency Response Sprint

**Objective**: Pass through gate, circle yellow buoy in direction indicated by color indicator, exit through gate.

**Computer Vision Requirements:**

**Primary Detections:**
- **Gate Buoys**: Red (class_id: 3) and green (class_id: 1) buoys (Polyform A-2, 1ft height)
- **Yellow Buoy** (class_id: 5): Target buoy to circle (Polyform A-2, 1ft height)
- **Color Indicator Buoy**: Red or green indicator determining circling direction
  - Red = circle counter-clockwise (right side)
  - Green = circle clockwise (left side)

**Vision Processing:**

1. **Gate Detection and Transit**:
   - Detect entry gate (red/green buoys)
   - Navigate through gate centerline
   - Detect exit gate for return path

2. **Yellow Buoy Detection and Tracking**:
   - Detect yellow buoy in field of view
   - Track yellow buoy position continuously
   - Estimate distance and bearing for approach

3. **Color Indicator Classification**:
   - Detect color indicator on custom buoy
   - Classify as red or green (determines circling direction)
   - Verify indicator state before initiating circle maneuver

4. **Circling Maneuver Guidance**:
   - Monitor ASV position relative to yellow buoy
   - Provide real-time feedback for circle completion
   - Verify correct direction (clockwise vs counter-clockwise)

**Output Requirements:**
- Gate buoy detections (entry and exit)
- Yellow buoy detection with position tracking
- Color indicator detection and classification (red/green)
- Real-time position feedback during circling maneuver
- Completion verification

**Performance Targets:**
- Yellow buoy detection range: >25 meters
- Color indicator classification: >98% accuracy (critical for direction)
- Update rate: ≥30 Hz for high-speed maneuvering
- Low latency: <50ms end-to-end for responsive control

---

### Task 4: Supply Drop

**Objective**: Deliver water to yellow vessels (black triangle target) or racquetball to black vessels (black plus/cross target).

**Computer Vision Requirements:**

**Primary Detections:**
- **Yellow Vessels** (class_id: 7 with shape detection): Stationary vessels with black triangle on both sides
- **Black Vessels** (class_id: 7 with shape detection): Stationary vessels with black plus/cross on both sides
- **Triangle Shapes** (class_id: 8): Black triangle targets on yellow vessels
- **Cross/Plus Shapes** (class_id: 6): Black plus targets on black vessels

**Task4 Supply Processor** (optional, `enable_task4:=true`): Subscribes to `/camera{N}/image_preprocessed` and `/camera{N}/detection_info`; publishes `/camera{N}/task4_detections` with `type` (`yellow_supply_drop`, `black_supply_drop`), `shape_bbox`, `vessel_bbox` (preprocessed frame), `source: "task4"`. Combiner merges these into `/combined/detection_info`; `bbox` in combined is in global frame (3×frame_width×frame_height).

**Vision Processing:**

1. **Vessel Detection**:
   - Detect yellow stationary vessels (up to 3)
   - Detect black stationary vessels (up to 3)
   - Estimate vessel position and orientation

2. **Target Shape Detection**:
   - **Yellow Vessels**: Detect black triangle shape on vessel side
   - **Black Vessels**: Detect black plus/cross shape on vessel side
   - Calculate target center coordinates for aiming

3. **Aiming and Delivery**:
   - Track target shape position in real-time
   - Calculate aim point for water stream or ball delivery
   - Verify target hit (water stream on triangle or ball in vessel)

4. **Multi-Vessel Management**:
   - Identify all available vessels (yellow and black)
   - Prioritize delivery order
   - Track completion status for each vessel

**Output Requirements:**
- Yellow vessel detection with position
- Black vessel detection with position
- Triangle shape detection on yellow vessels (target center)
- Plus/cross shape detection on black vessels (target center)
- Real-time target tracking for aiming
- Delivery verification (hit confirmation)

**Performance Targets:**
- Vessel detection range: >30 meters
- Shape detection accuracy: >90% (for precise aiming)
- Target center estimation: ±5cm accuracy (for water/ball delivery)
- Real-time tracking: ≥30 Hz for dynamic aiming

**Special Considerations:**
- Must detect shapes on both sides of vessels (approach from any angle)
- Robust detection in varying lighting conditions (water reflection)
- Handle partial occlusions (vessel may be partially visible)

---

### Task 5: Navigate the Marina

**Objective**: Dock in available slip with lowest number, indicated by green color indicator.

**Computer Vision Requirements:**

**Primary Detections:**
- **Dock Structure** (class_id: 7): Floating dock with multiple slips
- **Color Indicators**: Red or green indicators on dock slips
  - Green = available slip
  - Red = occupied slip
- **Number Signs**: Black number banners (1, 2, or 3) on dock slips
- **Stationary Vessels**: Yellow/black vessels in occupied slips

**Vision Processing:**

1. **Dock Detection and Approach**:
   - Detect dock structure in marina
   - Identify dock orientation and slip layout
   - Plan approach path to marina

2. **Slip Availability Detection**:
   - Detect color indicators on each slip
   - Classify as green (available) or red (occupied)
   - Identify all available slips

3. **Number Sign Recognition**:
   - Detect number banners on dock slips
   - Recognize numbers 1, 2, or 3
   - Associate numbers with slip positions

4. **Slip Selection Logic**:
   - Filter slips with green indicators (available)
   - Identify lowest number among available slips
   - Select target slip for docking

5. **Docking Guidance**:
   - Track target slip position
   - Provide real-time position feedback for docking maneuver
   - Verify successful docking in selected slip

**Output Requirements:**
- Dock structure detection
- Color indicator detection and classification (red/green) for each slip
- Number sign detection and recognition (1, 2, 3)
- Available slip identification with numbers
- Target slip selection (lowest number available)
- Real-time docking guidance

**Performance Targets:**
- Dock detection range: >40 meters
- Color indicator classification: >95% accuracy
- Number recognition accuracy: >90% (OCR or template matching)
- Slip identification: 100% accuracy (critical for correct docking)

**Special Considerations:**
- Must handle multiple slips simultaneously
- Robust number recognition in varying lighting/angles
- Handle cases where slip has red indicator but no vessel
- Accurate slip numbering is critical (docking in wrong slip = failure)

---

### Task 6: Harbor Alert

**Objective**: Detect audible signal and navigate to emergency zone (1 blast) or return to marina (2 blasts).

**Computer Vision Requirements:**

**Note**: This task is primarily audio-based (sound signal detection). Computer vision supports navigation to assigned zones but is not required for signal detection.

**Supporting Vision Requirements:**
- **Yellow Buoy Detection** (class_id: 5): Emergency response zone marker (Task 3 location)
- **Marina Detection** (class_id: 7): Dock structure for return navigation (Task 5 location)

**Vision Processing:**
1. **Emergency Zone Navigation** (1-blast signal):
   - Detect yellow buoy at Task 3 location
   - Navigate to yellow buoy position
   - Verify arrival at emergency zone

2. **Marina Return Navigation** (2-blast signal):
   - Detect dock structure (Task 5)
   - Navigate to marina entrance
   - Support docking if required

**Output Requirements:**
- Yellow buoy detection for emergency zone
- Dock detection for marina return
- Navigation guidance to assigned zone

**Performance Targets:**
- Detection range: >30 meters
- Fast response: <2 seconds from signal to navigation start

---

### Color Indicator Detection

**Overview**: Color indicators are 3D-printed cylinders that change between red and green, mounted on custom buoys. They are used across multiple tasks (Tasks 2, 3, 5) to provide dynamic state information.

**Technical Specifications:**
- **Appearance**: Single-colored cylinder (red OR green), visible 360° horizontally
- **Mounting**: Custom buoy base (~18 inches across)
- **State**: Binary (red or green, not both simultaneously)
- **Visibility**: Horizontal plane only (not visible from above/below)

**Computer Vision Approach:**

**Note**: Color indicator state detection is handled by a dedicated module (`task_vision_processor.py` - see separate documentation). The primary vision pipeline detects the custom buoy, and the task processor extracts and classifies the color indicator state.

**Detection Pipeline:**

1. **Buoy Detection**:
   - Primary vision pipeline detects custom buoy (class_id: 7 or specialized class)
   - Identifies bounding box of buoy structure

2. **Color Indicator Extraction**:
   - Extract region of interest (ROI) containing color indicator cylinder
   - Apply color space analysis (HSV recommended for robustness)
   - Handle varying lighting conditions (sunlight, water reflection)

3. **Color Classification**:
   - Classify indicator as red or green
   - Use temporal smoothing to handle transient detection errors
   - Maintain state history for robust classification

4. **State Reporting**:
   - Publish indicator state (red/green) to task-specific topics
   - Include confidence score for state classification
   - Report position of indicator for task execution

**Challenges and Solutions:**

**Challenge 1: Lighting Variations**
- **Solution**: Use HSV color space, adaptive thresholds, histogram analysis
- **Solution**: Temporal smoothing with state history (similar to light detection approach)

**Challenge 2: Partial Occlusion**
- **Solution**: Multi-camera fusion increases detection probability
- **Solution**: Confidence-based state reporting

**Challenge 3: Similar Colors (Red vs Green)**
- **Solution**: Precise color space thresholds
- **Solution**: Machine learning classifier if needed

**Performance Requirements:**
- Classification accuracy: >95% (critical for task decisions)
- Update rate: ≥10 Hz (sufficient for task execution)
- Detection range: >25 meters
- Robust to lighting: Works in direct sunlight, overcast, water reflection

**Integration:**
- Color indicator detection runs as separate processing module
- Subscribes to `/combined/detection_info` for buoy detections
- Publishes to `/task/color_indicator_state` with indicator states
- Task execution nodes subscribe to indicator states for decision-making

---

### Task Integration Summary

**Common Vision Capabilities Across Tasks:**

| Object Type | Class ID | Tasks Using | Critical Requirements |
|-------------|----------|-------------|----------------------|
| Red Buoy | 3 | 1, 2, 3 | Gate detection, navigation |
| Green Buoy | 1 | 1, 2, 3 | Gate detection, navigation |
| Black Buoy | 0 | 2 | Obstacle avoidance |
| Yellow Buoy | 5 | 3, 6 | Target identification |
| Yellow Vessel | 7 | 4 | Supply delivery target |
| Black Vessel | 7 | 4 | Supply delivery target |
| Dock | 7 | 5 | Marina navigation |
| Triangle | 8 | 4 | Water delivery target |
| Cross/Plus | 6 | 4 | Ball delivery target |
| Color Indicator | Special | 2, 3, 5 | Dynamic state detection |

**Task Execution Flow:**

```
Vision Pipeline → /combined/detection_info (bbox in global frame 3×frame_width×frame_height; global_frame; Task4: source "task4", type yellow/black_supply_drop)
                    ↓
        Task-Specific Processors
                    ↓
    Task Execution Nodes (Navigation, Delivery, etc.)
```

**Key Design Decisions:**

1. **Separation of Concerns**: Primary vision pipeline focuses on object detection; task-specific processors handle task logic
2. **Modularity**: Each task can be developed/tested independently
3. **Reusability**: Common objects (buoys, vessels) detected once, used by multiple tasks
4. **Real-time Performance**: Combined detections enable fast task execution decisions

---

## Monitoring and Diagnostics

### Health Monitoring

#### Topic Frequency Monitoring

**Command**: `ros2 topic hz /topic_name`

**Expected Frequencies**:
- `/camera{N}/image_raw`: ~15 Hz
- `/camera{N}/image_preprocessed`: ~15 Hz
- `/camera{N}/detections`: ~15 Hz
- `/camera{N}/detection_info`: ~15 Hz
- `/combined/detection_info`: ~15 Hz

**Alert Thresholds**:
- Camera topics: Alert if <30 Hz for >5 seconds
- Detection topics: Alert if <20 Hz for >5 seconds
- Combined topic: Alert if <25 Hz for >5 seconds

#### Node Status Monitoring

**Command**: `ros2 node list`

**Expected Nodes**:
- `/camera0/v4l2_camera_node`
- `/camera1/v4l2_camera_node`
- `/camera2/v4l2_camera_node`
- `/preprocess_camera0`
- `/preprocess_camera1`
- `/preprocess_camera2`
- `/inference_node_camera0`
- `/inference_node_camera1`
- `/inference_node_camera2`
- `/detection_combiner`

#### Resource Monitoring

**GPU Monitoring**:
```bash
# Continuous monitoring
watch -n 1 nvidia-smi

# Memory usage
nvidia-smi --query-gpu=memory.used,memory.total --format=csv

# Temperature
nvidia-smi --query-gpu=temperature.gpu --format=csv
```

**CPU Monitoring**:
```bash
# Per-core usage
htop

# Process-specific
top -p $(pgrep -f vision_inference)
```

**Memory Monitoring**:
```bash
# System memory
free -h

# Process memory
ps aux | grep vision
```

### Logging

#### Log Locations

- ROS2 logs: `~/.ros/log/`
- Application logs: `/var/log/vision_pipeline/` (if configured)
- System logs: `/var/log/syslog`

#### Log Levels

- **DEBUG**: Detailed debugging information
- **INFO**: General operational messages
- **WARN**: Warning messages (non-critical issues)
- **ERROR**: Error messages (recoverable failures)
- **FATAL**: Critical errors (node crashes)

#### Log Rotation

Configure log rotation to prevent disk fill:
```bash
# /etc/logrotate.d/vision-pipeline
/var/log/vision_pipeline/*.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
}
```

### Diagnostic Tools

#### Custom Diagnostic Node (To Be Created)

**Purpose**: Centralized health monitoring and alerting

**Functionality**:
- Monitor all topics and nodes
- Track performance metrics
- Generate alerts for failures
- Publish health status topic

**Output Topic**: `/vision_pipeline/health_status`

```json
{
  "timestamp": 1234567890.123,
  "status": "healthy",
  "cameras": {
    "0": {"status": "active", "fps": 15.0},
    "1": {"status": "active", "fps": 14.9},
    "2": {"status": "active", "fps": 15.0}
  },
  "nodes": {
    "preprocess_camera0": "running",
    "inference_node_camera0": "running",
    ...
  },
  "resources": {
    "gpu_memory_mb": 1500,
    "gpu_utilization": 65.0,
    "cpu_utilization": 45.0
  },
  "alerts": []
}
```

### Performance Profiling

#### Inference Profiling

**Tool**: TensorRT Profiler
```bash
# Enable profiling in inference node
# Log inference times to file
# Analyze with custom scripts
```

#### ROS2 Profiling

**Tool**: `ros2 trace`
```bash
# Record trace
ros2 trace --session-name vision_pipeline

# Analyze trace
ros2 trace analyze vision_pipeline
```

---

## Appendices

### Appendix A: Message Type Definitions

**sensor_msgs/Image** (Standard ROS2):
- Full specification: http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html

**std_msgs/String**:
- Simple string message for JSON data
- Specification: http://docs.ros.org/en/api/std_msgs/html/msg/String.html

### Appendix B: Class ID Mapping

**Maritime Object Detection Class IDs**:

| Class ID | Class Name | Category | Dataset Count |
|----------|------------|----------|---------------|
| 0 | black_buoy | Buoy | 4,931 |
| 1 | green_buoy | Buoy | 9,007 |
| 2 | green_pole_buoy | Buoy | 7,533 |
| 3 | red_buoy | Buoy | 8,760 |
| 4 | red_pole_buoy | Buoy | 6,767 |
| 5 | yellow_buoy | Buoy | 4,938 |
| 6 | cross | Navigation Marker | 3,311 |
| 7 | dock | Infrastructure | 4,991 |
| 8 | triangle | Navigation Marker | 3,345 |

**Total Classes**: 9

**Configuration File**: `config/class_mapping.yaml`

**Custom Mapping**: Define in model training documentation

### Appendix C: Troubleshooting Guide

**Issue**: Camera not detected
- Check: `ls -l /dev/video*`
- Check: USB connection
- Check: `v4l2-ctl --list-devices`

**Issue**: Low FPS
- Check: GPU utilization
- Check: CPU utilization
- Check: USB bandwidth
- Reduce resolution if necessary

**Issue**: High latency
- Check: Inference time (should be <20 ms)
- Check: Network congestion
- Check: System load

**Issue**: No detections
- Check: Confidence threshold (may be too high)
- Check: Model accuracy
- Check: Input image quality

---

## Document Version History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-01-XX | System Design | Initial comprehensive design document |

---

## References

- ROS2 Documentation: https://docs.ros.org/
- TensorRT Developer Guide: https://docs.nvidia.com/deeplearning/tensorrt/
- NVIDIA Jetson Documentation: https://developer.nvidia.com/embedded/jetson
- V4L2 Camera Driver: https://github.com/ros-drivers/v4l2_camera
- YOLO Documentation: https://github.com/ultralytics/ultralytics

---

**End of Document**
