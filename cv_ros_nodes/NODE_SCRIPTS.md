# ROS2 Node Scripts

## Pipeline Architecture (3 Cameras - Parallel Processing)

**Recommended: Independent pipelines per camera for fault isolation and scalability**

```
Camera0: /camera0/image_raw → preprocessing0 → /camera0/image_preprocessed → inference0 → /camera0/detections
Camera1: /camera1/image_raw → preprocessing1 → /camera1/image_preprocessed → inference1 → /camera1/detections  
Camera2: /camera2/image_raw → preprocessing2 → /camera2/image_preprocessed → inference2 → /camera2/detections
                                                                                            ↓
                                                                                    combiner → /combined/detection_info
```

**Benefits:**
- Fault isolation: one camera failure doesn't affect others
- Independent frame rates per camera
- Easy to scale and tune individually
- Simple, maintainable code
- Combined output for unified detection processing

## Camera Nodes
```bash
# Start all 3 camera nodes
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -p image_size:="[1920,1200]" -p framerate:=60.0 -r __ns:=/camera0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video1 -p image_size:="[1920,1200]" -p framerate:=60.0 -r __ns:=/camera1
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video2 -p image_size:="[1920,1200]" -p framerate:=60.0 -r __ns:=/camera2
```

## CV Nodes (Per Camera)
```bash
# Preprocessing nodes (one per camera)
python3 vision_preprocessing.py --camera_id 0  # /camera0/image_raw → /camera0/image_preprocessed
python3 vision_preprocessing.py --camera_id 1  # /camera1/image_raw → /camera1/image_preprocessed
python3 vision_preprocessing.py --camera_id 2  # /camera2/image_raw → /camera2/image_preprocessed

# Inference nodes (one per camera)
python3 vision_inference.py --camera_id 0  # /camera0/image_preprocessed → /camera0/detections
python3 vision_inference.py --camera_id 1  # /camera1/image_preprocessed → /camera1/detections
python3 vision_inference.py --camera_id 2  # /camera2/image_preprocessed → /camera2/detections

# Combiner node (combines all camera detections)
python3 vision_combiner.py  # Subscribes: /camera0/detection_info, /camera1/detection_info, /camera2/detection_info
                            # Publishes: /combined/detection_info

# Optional: Apply NMS across cameras to remove duplicates
python3 vision_combiner.py --apply_nms --iou_threshold 0.5
```

## Monitoring
```bash
# Check all camera feeds
ros2 topic hz /camera0/image_raw
ros2 topic hz /camera1/image_raw
ros2 topic hz /camera2/image_raw

# Check preprocessing output
ros2 topic hz /camera0/image_preprocessed
ros2 topic hz /camera1/image_preprocessed
ros2 topic hz /camera2/image_preprocessed

# Check detections
ros2 topic hz /camera0/detections
ros2 topic hz /camera1/detections
ros2 topic hz /camera2/detections

# Check detection info (JSON)
ros2 topic echo /camera0/detection_info
ros2 topic echo /camera1/detection_info
ros2 topic echo /camera2/detection_info

# Check combined detections
ros2 topic hz /combined/detection_info
ros2 topic echo /combined/detection_info

# List all topics
ros2 topic list
```

## Combined Detection Output Format

The combiner node publishes to `/combined/detection_info` with the following JSON structure:

```json
{
  "timestamp": 1234567890.123,
  "num_cameras": 3,
  "total_detections": 5,
  "camera_stats": {
    "0": {"status": "active", "num_detections": 2, "fps": 30.5, "timestamp": 1234567890.1},
    "1": {"status": "active", "num_detections": 2, "fps": 29.8, "timestamp": 1234567890.1},
    "2": {"status": "active", "num_detections": 1, "fps": 30.2, "timestamp": 1234567890.1}
  },
  "detections": [
    {
      "camera_id": 0,
      "class_id": 2,
      "score": 0.85,
      "bbox": [100, 150, 200, 250]
    },
    ...
  ]
}
```

Each detection includes its source `camera_id` for tracking which camera detected each object.
