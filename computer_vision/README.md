# Computer Vision ROS2 Pipeline

A multi-camera object detection system using TensorRT-optimized YOLO models on ROS2 Humble. This pipeline processes three camera feeds in parallel, performs real-time inference, and combines detections into a unified output.

![Pipeline Architecture](docs/pipeline_architecture.png)
*Figure 1: Multi-camera detection pipeline architecture*

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation & Setup](#installation--setup)
- [Running Nodes](#running-nodes)
- [Launch Files](#launch-files)
- [Topics & Messages](#topics--messages)


---

## Overview

This project implements a real-time object detection pipeline for autonomous systems using:

- **3x USB Cameras** (1920x1200 @ 30fps) via `v4l2_camera`
- **TensorRT Inference** (GPU-accelerated YOLO detection)
- **ROS2 Humble** (distributed processing)
- **Multi-camera fusion** (combines detections from all cameras)

### Key Features

- ✅ Parallel processing of 3 camera feeds
- ✅ GPU-accelerated inference with TensorRT
- ✅ Staleness filtering (detects camera failures)
- ✅ Timestamp synchronization
- ✅ Detailed detection metadata (bbox, class, confidence)
- ✅ Health monitoring per camera

---

## Architecture

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│  Camera 0   │────▶│ Preprocess 0 │────▶│ Inference 0 │────▶│              │
│ /dev/video0 │     │  640x480     │     │ TensorRT    │     │              │
└─────────────┘     └──────────────┘     └─────────────┘     │              │
                                                              │   Combiner   │
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     │              │
│  Camera 1   │────▶│ Preprocess 1 │────▶│ Inference 1 │────▶│  /combined/  │
│ /dev/video2 │     │  640x480     │     │ TensorRT    │     │detection_info│
└─────────────┘     └──────────────┘     └─────────────┘     │              │
                                                              │              │
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     │              │
│  Camera 2   │────▶│ Preprocess 2 │────▶│ Inference 2 │────▶│              │
│ /dev/video4 │     │  640x480     │     │ TensorRT    │     └──────────────┘
└─────────────┘     └──────────────┘     └─────────────┘
```

**Pipeline Flow:**
1. **Cameras** (`v4l2_camera_node`) → `/camera{N}/image_raw`
2. **Preprocessing** (`vision_preprocessing`) → `/camera{N}/image_preprocessed` (resize to 640x480)
3. **Inference** (`vision_inference`) → `/camera{N}/detections` + `/camera{N}/detection_info`
4. **Combiner** (`vision_combiner`) → `/combined/detection_info`

---

## Prerequisites

### System Requirements

- **OS**: Ubuntu 22.04 (for ROS2 Humble)
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **GPU**: NVIDIA GPU with CUDA support
- **Python Packages**:
  ```bash
  pip install numpy<2 opencv-python tensorrt pycuda
  ```

### ROS2 Packages

```bash
sudo apt install ros-humble-v4l2-camera ros-humble-cv-bridge
```

### Hardware Setup

- 3x USB cameras connected to `/dev/video0`, `/dev/video2`, `/dev/video4`
- Camera resolution: 1920x1200 @ 30fps

---

## Installation & Setup

### 1. Build the ROS2 Package

The package is already configured with all nodes and dependencies. Build it from the workspace root:

```bash
cd ~/autonomy-ws-25-26/computer_vision

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the package
colcon build --packages-select cv_ros_nodes

# Source the workspace
source install/setup.bash
```

**Note**: The workspace root is `~/autonomy-ws-25-26/computer_vision/`, and the package is located at `src/cv_ros_nodes/`.

---

## Running Nodes

> **Note**: For detailed usage of individual nodes, see [NODE_SCRIPTS.md](NODE_SCRIPTS.md).

### ROS2 Executables

After building with `colcon build`, use ROS2 executables:

```bash
# Source workspace first
cd ~/autonomy-ws-25-26/computer_vision
source install/setup.bash

# Preprocessing
ros2 run cv_ros_nodes vision_preprocessing --camera_id 0

# Inference
ros2 run cv_ros_nodes vision_inference \
  --camera_id 0 \
  --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine \
  --conf_threshold 0.25

# Combiner
ros2 run cv_ros_nodes vision_combiner --staleness_threshold 1.0

# Camera Viewer (save images to disk)
ros2 run cv_ros_nodes camera_viewer --camera_id 0 --output_dir ./camera_images
```

---

## Launch Files

### Quick Start: Launch Everything (Recommended)

The launch file starts all nodes automatically - this is the recommended way to run the pipeline:

```bash
cd ~/autonomy-ws-25-26/computer_vision
source install/setup.bash

# Launch all nodes
ros2 launch cv_ros_nodes launch_cv.py
```

### Launch File Location

The launch file is located at:
- **Source**: `~/autonomy-ws-25-26/computer_vision/src/cv_ros_nodes/cv_ros_nodes/launch/launch_cv.py`
- **After build**: `~/autonomy-ws-25-26/computer_vision/install/cv_ros_nodes/share/cv_ros_nodes/launch/launch_cv.py`

### Launch Arguments

Customize launch parameters:

```bash
# Custom engine path
ros2 launch cv_ros_nodes launch_cv.py \
  engine_path:=/path/to/model.engine

# Custom confidence threshold
ros2 launch cv_ros_nodes launch_cv.py \
  conf_threshold:=0.5

# Custom staleness threshold
ros2 launch cv_ros_nodes launch_cv.py \
  staleness_threshold:=0.5

# All together
ros2 launch cv_ros_nodes launch_cv.py \
  engine_path:=/path/to/model.engine \
  conf_threshold:=0.3 \
  staleness_threshold:=0.5
```

### Launch File Structure

The launch file (`launch_cv.py`) defines:

1. **Launch Arguments**:
   - `engine_path`: Path to TensorRT engine file
   - `conf_threshold`: Detection confidence threshold (0.0-1.0)
   - `staleness_threshold`: Max age for detections (seconds)

2. **Camera Nodes** (3x):
   - `v4l2_camera_node` for each camera
   - Namespaces: `/camera0`, `/camera1`, `/camera2`

3. **Preprocessing Nodes** (3x):
   - `vision_preprocessing` for each camera
   - Subscribes to `/camera{N}/image_raw`
   - Publishes to `/camera{N}/image_preprocessed`

4. **Inference Nodes** (3x):
   - `vision_inference` for each camera
   - Subscribes to `/camera{N}/image_preprocessed`
   - Publishes to `/camera{N}/detections` and `/camera{N}/detection_info`

5. **Combiner Node** (1x):
   - `vision_combiner`
   - Subscribes to all `/camera{N}/detection_info`
   - Publishes to `/combined/detection_info`


## Topics & Messages

### Topic List

```bash
# List all topics
ros2 topic list

# Monitor topic frequency
ros2 topic hz /camera0/image_raw
ros2 topic hz /camera0/image_preprocessed
ros2 topic hz /camera0/detection_info
ros2 topic hz /combined/detection_info

# View topic data
ros2 topic echo /camera0/detection_info
ros2 topic echo /combined/detection_info
```

### Topic Structure

| Topic | Type | Description |
|-------|------|-------------|
| `/camera{N}/image_raw` | `sensor_msgs/Image` | Raw camera feed (1920x1200) |
| `/camera{N}/image_preprocessed` | `sensor_msgs/Image` | Preprocessed image (640x480) |
| `/camera{N}/detections` | `sensor_msgs/Image` | Image with bounding boxes drawn |
| `/camera{N}/detection_info` | `std_msgs/String` | JSON detection metadata |
| `/combined/detection_info` | `std_msgs/String` | Combined detections from all cameras |

### Detection Info Format

**Per-Camera Detection Info** (`/camera{N}/detection_info`):

```json
{
  "camera_id": 0,
  "timestamp": 1234567890.123,
  "frame_id": "camera0",
  "num_detections": 2,
  "inference_time_ms": 12.5,
  "fps": 30.2,
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

**Combined Detection Info** (`/combined/detection_info`):

```json
{
  "timestamp": 1234567890.123,
  "num_cameras": 3,
  "num_active_cameras": 2,
  "num_stale_cameras": 1,
  "num_no_data_cameras": 0,
  "staleness_threshold": 1.0,
  "total_detections": 5,
  "camera_stats": {
    "0": {
      "status": "active",
      "num_detections": 2,
      "fps": 30.5,
      "timestamp": 1234567890.1,
      "time_since_update": 0.023
    },
    "1": {
      "status": "active",
      "num_detections": 2,
      "fps": 29.8,
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
      "x1": 100.0,
      "y1": 150.0,
      "x2": 200.0,
      "y2": 250.0,
      "width": 100.0,
      "height": 100.0
    }
  ]
}
```

## Project Structure

```
autonomy-ws-25-26/
├── cv_ros_nodes/              # Source package
│   ├── vision_preprocessing.py    # Image preprocessing node
│   ├── vision_inference.py        # TensorRT inference node
│   ├── vision_combiner.py         # Multi-camera combiner
│   ├── camera_viewer.py           # Image saver utility
│   ├── launch_cv.py               # Launch file
│   ├── model.engine               # TensorRT engine file
│   ├── package.xml                # ROS2 package manifest
│   └── setup.py                   # Python package setup
├── src/                        # ROS2 workspace
│   ├── build/                  # Build artifacts
│   ├── install/                # Installed packages
│   └── log/                    # Build logs
├── camera_images/              # Saved camera frames (from camera_viewer)
└── README.md                   # This file
```

---


## License

MIT License - See LICENSE file for details

---

## Contributors

- Lorenzo DeMarni

---

*Last Updated: January 2025*
