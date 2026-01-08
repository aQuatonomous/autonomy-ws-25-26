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
12. [Monitoring and Diagnostics](#monitoring-and-diagnostics)

---

## Executive Summary

This document describes the complete design strategy for a multi-camera computer vision pipeline using ROS2 and TensorRT for real-time object detection. The system processes three synchronized camera feeds (video0, video1, video2) through independent preprocessing and inference pipelines, then combines detections into a unified output stream.

**Key Design Principles:**
- **Fault Isolation**: Independent pipelines per camera ensure single camera failures don't cascade
- **Scalability**: Modular design allows easy addition/removal of cameras
- **Performance**: TensorRT optimization for real-time inference on NVIDIA Jetson Orin
- **Reliability**: Robust error handling and graceful degradation

---

## System Architecture

### High-Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Hardware Layer                                   │
├─────────────────────────────────────────────────────────────────────────┤
│  Camera 0 (/dev/video0)  │  Camera 1 (/dev/video1)  │  Camera 2 (/dev/video2) │
│  1920x1200 @ 60 FPS      │  1920x1200 @ 60 FPS      │  1920x1200 @ 60 FPS     │
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
                    │      - Optional NMS across cameras              │
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
- Raw images captured at 1920x1200 resolution, 60 FPS
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
- Optional NMS to remove duplicates across camera views
- Output: Unified detection list with camera source tracking

---

## Node Architecture

### 1. Camera Driver Nodes (v4l2_camera_node)

**Node Name**: `v4l2_camera_node` (3 instances with different namespaces)

**Purpose**: Interface with V4L2-compatible USB cameras

**Instances**:
- `/camera0/v4l2_camera_node` (video0)
- `/camera1/v4l2_camera_node` (video1)
- `/camera2/v4l2_camera_node` (video2)

**Parameters**:
- `video_device`: `/dev/video0`, `/dev/video1`, `/dev/video2`
- `image_size`: `[1920, 1200]`
- `framerate`: `60.0`
- `namespace`: `/camera0`, `/camera1`, `/camera2`

**Subscriptions**: None (hardware interface)

**Publications**:
- `/camera{N}/image_raw` (sensor_msgs/Image)
  - Encoding: `bgr8`
  - Resolution: 1920x1200
  - Frame rate: ~60 Hz
  - Header includes timestamp and frame_id

**Launch Command**:
```bash
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:="[1920,1200]" \
  -p framerate:=60.0 \
  -r __ns:=/camera0
```

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
  - Resolution: 640x480 (configurable)
  - Frame rate: Matches input (up to 60 Hz)
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
python3 vision_preprocessing.py --camera_id 0
python3 vision_preprocessing.py --camera_id 1
python3 vision_preprocessing.py --camera_id 2
```

---

### 3. Inference Nodes (vision_inference.py)

**Node Name**: `inference_node_camera{N}` (N = 0, 1, 2)

**Purpose**: Run TensorRT-optimized YOLO inference on preprocessed images

**Class**: `InferenceNode`

**Dependencies**:
- TensorRT 10.3.0
- CUDA 12.6
- NVIDIA Jetson Orin GPU

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
  - Resolution: 640x480 (matches input)
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
python3 vision_inference.py --camera_id 0 --engine_path model.engine --conf_threshold 0.25
python3 vision_inference.py --camera_id 1 --engine_path model.engine --conf_threshold 0.25
python3 vision_inference.py --camera_id 2 --engine_path model.engine --conf_threshold 0.25
```

---

### 4. Detection Combiner Node (vision_combiner.py)

**Node Name**: `detection_combiner`

**Purpose**: Aggregate detections from all cameras into unified output

**Class**: `DetectionCombiner`

**Parameters** (Command Line):
- `--apply_nms`: Boolean flag - Apply NMS across cameras to remove duplicates
- `--iou_threshold`: Float (default: 0.5) - IoU threshold for NMS (0.0-1.0)

**Subscriptions**:
- `/camera0/detection_info` (std_msgs/String) - Queue size: 10
- `/camera1/detection_info` (std_msgs/String) - Queue size: 10
- `/camera2/detection_info` (std_msgs/String) - Queue size: 10

**Publications**:
- `/combined/detection_info` (std_msgs/String)
  - Format: JSON
  - Frequency: ~30 Hz (timer-based, also publishes on new data)
  - Content: Combined detections with camera source tracking

**Functionality**:

**a) Detection Aggregation**:
- Maintains latest detections from each camera
- Tracks update timestamps for staleness detection
- Combines all active detections into single list

**b) Optional Cross-Camera NMS**:
- Calculates IoU between detections from different cameras
- Removes overlapping detections (keeps highest confidence)
- Useful when cameras have overlapping fields of view

**c) Health Monitoring**:
- Tracks camera status: `active`, `no_data`, `stale`
- Reports FPS and detection counts per camera
- Timeout: 1.0 second (configurable)

**d) Temporal Smoothing**:
- Timer-based publishing ensures output even if cameras are slow
- Prevents gaps in detection stream

**Performance Characteristics**:
- Processing time: <1 ms per update
- CPU-bound operation
- Minimal memory footprint

**Launch Command**:
```bash
# Basic mode (no NMS)
python3 vision_combiner.py

# With NMS to remove duplicates
python3 vision_combiner.py --apply_nms --iou_threshold 0.5
```

---

## Topic Communication Matrix

### Complete Topic List

| Topic Name | Message Type | Publisher Node | Subscriber Nodes | Frequency | Purpose |
|------------|--------------|----------------|------------------|-----------|---------|
| `/camera0/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera0) | preprocess_camera0 | ~60 Hz | Raw camera feed 0 |
| `/camera1/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera1) | preprocess_camera1 | ~60 Hz | Raw camera feed 1 |
| `/camera2/image_raw` | sensor_msgs/Image | v4l2_camera_node (camera2) | preprocess_camera2 | ~60 Hz | Raw camera feed 2 |
| `/camera0/image_preprocessed` | sensor_msgs/Image | preprocess_camera0 | inference_node_camera0 | ~60 Hz | Preprocessed images 0 |
| `/camera1/image_preprocessed` | sensor_msgs/Image | preprocess_camera1 | inference_node_camera1 | ~60 Hz | Preprocessed images 1 |
| `/camera2/image_preprocessed` | sensor_msgs/Image | preprocess_camera2 | inference_node_camera2 | ~60 Hz | Preprocessed images 2 |
| `/camera0/detections` | sensor_msgs/Image | inference_node_camera0 | (monitoring/visualization) | ~30-60 Hz | Detection visualization 0 |
| `/camera1/detections` | sensor_msgs/Image | inference_node_camera1 | (monitoring/visualization) | ~30-60 Hz | Detection visualization 1 |
| `/camera2/detections` | sensor_msgs/Image | inference_node_camera2 | (monitoring/visualization) | ~30-60 Hz | Detection visualization 2 |
| `/camera0/detection_info` | std_msgs/String | inference_node_camera0 | detection_combiner | ~30-60 Hz | Detection metadata 0 |
| `/camera1/detection_info` | std_msgs/String | inference_node_camera1 | detection_combiner | ~30-60 Hz | Detection metadata 1 |
| `/camera2/detection_info` | std_msgs/String | inference_node_camera2 | detection_combiner | ~30-60 Hz | Detection metadata 2 |
| `/combined/detection_info` | std_msgs/String | detection_combiner | (downstream nodes) | ~30 Hz | Unified detection output |

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
  - `bbox`: Bounding box `[x1, y1, x2, y2]` in pixel coordinates

**Combined Detection Info** (`/combined/detection_info`):

```json
{
  "timestamp": 1234567890.123456,
  "num_cameras": 3,
  "total_detections": 8,
  "camera_stats": {
    "0": {
      "status": "active",
      "num_detections": 3,
      "fps": 32.5,
      "timestamp": 1234567890.123
    },
    "1": {
      "status": "active",
      "num_detections": 2,
      "fps": 31.8,
      "timestamp": 1234567890.124
    },
    "2": {
      "status": "stale",
      "num_detections": 0,
      "time_since_update": 1.5
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
    },
    ...
  ]
}
```

**Field Descriptions**:
- `timestamp`: Current system timestamp
- `num_cameras`: Total number of cameras (always 3)
- `total_detections`: Total number of detections across all cameras
- `camera_stats`: Per-camera health and statistics
  - `status`: `"active"`, `"no_data"`, or `"stale"`
  - `num_detections`: Detections from this camera
  - `fps`: Camera processing rate
  - `timestamp`: Last update timestamp (if active)
  - `time_since_update`: Seconds since last update (if stale)
- `detections`: Combined array with `camera_id` field added to each detection

---

## TensorRT Model Training Pipeline

### Overview

The model training pipeline converts a trained YOLO model (PyTorch/ONNX) into an optimized TensorRT engine for deployment on NVIDIA Jetson Orin.

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

**Process**:
```python
import torch

# Load trained model
model = torch.load('weights.pt')
model.eval()

# Export to ONNX
dummy_input = torch.randn(1, 3, 640, 640)
torch.onnx.export(
    model,
    dummy_input,
    "weights.onnx",
    input_names=['images'],
    output_names=['output'],
    dynamic_axes=None,  # Fixed input size for TensorRT
    opset_version=11
)
```

**Validation**:
- Verify ONNX model loads correctly
- Test inference on sample images
- Compare outputs with PyTorch model

**Tools**: PyTorch ONNX export, ONNX Runtime for validation

#### Stage 3: TensorRT Engine Conversion

**Input**: ONNX model (`.onnx`)
**Output**: TensorRT engine (`.engine`)

**Method 1: Using trtexec (Recommended)**

```bash
# Basic conversion (FP32)
/usr/src/tensorrt/bin/trtexec \
  --onnx=weights.onnx \
  --saveEngine=model.engine \
  --workspace=4096

# FP16 precision (faster, minimal accuracy loss)
/usr/src/tensorrt/bin/trtexec \
  --onnx=weights.onnx \
  --saveEngine=model_fp16.engine \
  --fp16 \
  --workspace=4096

# INT8 precision (fastest, requires calibration)
/usr/src/tensorrt/bin/trtexec \
  --onnx=weights.onnx \
  --saveEngine=model_int8.engine \
  --int8 \
  --workspace=4096 \
  --calib=<calibration_cache>
```

**Method 2: Python API (For Custom Optimization)**

```python
import tensorrt as trt

# Create builder and network
logger = trt.Logger(trt.Logger.WARNING)
builder = trt.Builder(logger)
network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
parser = trt.OnnxParser(network, logger)

# Parse ONNX model
with open("weights.onnx", "rb") as model:
    parser.parse(model.read())

# Configure builder
config = builder.create_builder_config()
config.max_workspace_size = 4 * 1024 * 1024 * 1024  # 4 GB
config.set_flag(trt.BuilderFlag.FP16)  # Enable FP16

# Build engine
engine = builder.build_engine(network, config)

# Save engine
with open("model.engine", "wb") as f:
    f.write(engine.serialize())
```

**Precision Selection Guide**:
- **FP32**: Highest accuracy, slowest (baseline)
- **FP16**: ~2x faster, <1% accuracy loss (recommended for Jetson)
- **INT8**: ~4x faster, requires calibration dataset, 1-3% accuracy loss

**Optimization Parameters**:
- `workspace`: GPU memory for optimization (4096 MB recommended)
- `minShapes`: Minimum input dimensions
- `optShapes`: Optimal input dimensions (640x640)
- `maxShapes`: Maximum input dimensions

#### Stage 4: Engine Validation

**Input**: TensorRT engine (`.engine`)
**Output**: Validation report

**Process**:
```python
# Test inference on validation set
from test_inference import TensorRTInference

inferencer = TensorRTInference("model.engine")

# Run on test images
for image_path in test_images:
    detections = test_single_image(image_path, "model.engine")
    # Compare with ground truth
    # Calculate mAP, precision, recall
```

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
- Engine file: `/home/lorenzo/autonomy-ws-25-26/cv_ros_nodes/model.engine`
- ONNX file: `/home/lorenzo/autonomy-ws-25-26/model_testing_training/weights.onnx`
- PyTorch weights: `/home/lorenzo/autonomy-ws-25-26/model_testing_training/weights.pt`

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
- **TC-PREP-001**: Verify image resize (1920x1200 → 640x480)
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
- **TC-COMB-004**: Verify NMS functionality (if enabled)
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
- Run pipeline at maximum camera frame rate (60 FPS)
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
| Camera capture | <16.7 ms (60 FPS) | Hardware dependent |
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

**Primary Platform**: NVIDIA Jetson Orin
- GPU: NVIDIA Orin (2048 CUDA cores)
- CPU: ARM Cortex-A78AE (12 cores)
- Memory: 32 GB LPDDR5
- Storage: 64 GB+ NVMe SSD
- OS: Ubuntu 20.04 / JetPack 5.x

**Camera Requirements**:
- 3x USB 3.0 cameras
- Resolution: 1920x1200 minimum
- Frame rate: 60 FPS capable
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
/home/lorenzo/autonomy-ws-25-26/
├── cv_ros_nodes/
│   ├── vision_preprocessing.py
│   ├── vision_inference.py
│   ├── vision_combiner.py
│   ├── model.engine
│   └── launch/
│       └── vision_pipeline.launch.py
├── model_testing_training/
│   ├── test_inference.py
│   ├── weights.pt
│   └── weights.onnx
└── config/
    └── vision_config.yaml
```

### Launch Configuration

**Single Launch File** (`vision_pipeline.launch.py`):

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Camera nodes
        ExecuteProcess(
            cmd=['ros2', 'run', 'v4l2_camera', 'v4l2_camera_node',
                 '--ros-args', '-p', 'video_device:=/dev/video0',
                 '-p', 'image_size:="[1920,1200]"', '-p', 'framerate:=60.0',
                 '-r', '__ns:=/camera0'],
            name='camera0'
        ),
        # ... (camera1, camera2)
        
        # Preprocessing nodes
        Node(
            package='cv_ros_nodes',
            executable='vision_preprocessing.py',
            name='preprocess_camera0',
            arguments=['--camera_id', '0']
        ),
        # ... (camera1, camera2)
        
        # Inference nodes
        Node(
            package='cv_ros_nodes',
            executable='vision_inference.py',
            name='inference_camera0',
            arguments=['--camera_id', '0', '--engine_path', 'model.engine']
        ),
        # ... (camera1, camera2)
        
        # Combiner node
        Node(
            package='cv_ros_nodes',
            executable='vision_combiner.py',
            name='detection_combiner'
        )
    ])
```

**Launch Command**:
```bash
ros2 launch cv_ros_nodes vision_pipeline.launch.py
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
ExecStart=/usr/bin/python3 /home/lorenzo/autonomy-ws-25-26/cv_ros_nodes/launch/vision_pipeline.launch.py
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
  output_resolution: [640, 480]
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
  apply_nms: false
  iou_threshold: 0.5
  detection_timeout: 1.0  # seconds
  publish_rate: 30.0  # Hz
  queue_size: 10
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

---

## Monitoring and Diagnostics

### Health Monitoring

#### Topic Frequency Monitoring

**Command**: `ros2 topic hz /topic_name`

**Expected Frequencies**:
- `/camera{N}/image_raw`: ~60 Hz
- `/camera{N}/image_preprocessed`: ~60 Hz
- `/camera{N}/detections`: ~30-60 Hz
- `/camera{N}/detection_info`: ~30-60 Hz
- `/combined/detection_info`: ~30 Hz

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
    "0": {"status": "active", "fps": 60.0},
    "1": {"status": "active", "fps": 59.8},
    "2": {"status": "active", "fps": 60.1}
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

### Appendix B: TensorRT Engine Specifications

**Input**:
- Shape: `[1, 3, 640, 640]`
- Type: `float32`
- Format: CHW (Channel, Height, Width)
- Normalization: [0.0, 1.0]

**Output**:
- Shape: `[1, 27, 8400]` (example for YOLO)
- Type: `float32`
- Format: Detection predictions

### Appendix C: Class ID Mapping

**Example Class IDs** (YOLO COCO dataset):
- 0: person
- 1: bicycle
- 2: car
- 3: motorcycle
- 4: airplane
- 5: bus
- ... (80 total classes)

**Custom Mapping**: Define in model training documentation

### Appendix D: Troubleshooting Guide

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
