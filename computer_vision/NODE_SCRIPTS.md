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


## Camera Nodes
```bash
# Start all 3 camera nodes
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -p image_size:="[1920,1200]" -p framerate:=30.0 -r __ns:=/camera0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video2 -p image_size:="[1920,1200]" -p framerate:=30.0 -r __ns:=/camera1
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video4 -p image_size:="[1920,1200]" -p framerate:=30.0 -r __ns:=/camera2
```

## CV Nodes (Per Camera)
```bash
# Preprocessing nodes (one per camera)
ros2 run cv_ros_nodes vision_preprocessing --camera_id 0  # /camera0/image_raw → /camera0/image_preprocessed
ros2 run cv_ros_nodes vision_preprocessing --camera_id 1  # /camera1/image_raw → /camera1/image_preprocessed
ros2 run cv_ros_nodes vision_preprocessing --camera_id 2  # /camera2/image_raw → /camera2/image_preprocessed

# Inference nodes (one per camera)
ros2 run cv_ros_nodes vision_inference --camera_id 0 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine
ros2 run cv_ros_nodes vision_inference --camera_id 1 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine
ros2 run cv_ros_nodes vision_inference --camera_id 2 --engine_path ~/autonomy-ws-25-26/computer_vision/cv_scripts/model.engine

# Combiner node (combines all camera detections)
ros2 run cv_ros_nodes vision_combiner  # Subscribes: /camera0/detection_info, /camera1/detection_info, /camera2/detection_info
                                        # Publishes: /combined/detection_info
                                        # Default: Latest value approach with 1.0s staleness threshold

# Custom staleness threshold (exclude cameras that haven't updated recently)
ros2 run cv_ros_nodes vision_combiner --staleness_threshold 0.5  # More strict (0.5s)
ros2 run cv_ros_nodes vision_combiner --staleness_threshold 2.0  # More lenient (2.0s)

# Enable overlap deduplication (recommended for side-by-side cameras with 15° overlap)
ros2 run cv_ros_nodes vision_combiner --deduplicate_overlap

# Overlap deduplication with custom parameters
ros2 run cv_ros_nodes vision_combiner --deduplicate_overlap --overlap_zone_width 0.20 --overlap_y_tolerance 0.15

# Optional: Use timestamp-based synchronization (more complex)
ros2 run cv_ros_nodes vision_combiner --use_timestamp_sync --sync_window 0.05
```

## Camera Viewer Node (Image Saving)

The camera viewer node saves camera images to disk for debugging, monitoring, or offline analysis.

```bash
# View camera 0 (default, saves to ./camera_images/)
ros2 run cv_ros_nodes camera_viewer

# View a specific camera
ros2 run cv_ros_nodes camera_viewer --camera_id 0  # /camera0/image_raw → cam0.jpg
ros2 run cv_ros_nodes camera_viewer --camera_id 1  # /camera1/image_raw → cam1.jpg
ros2 run cv_ros_nodes camera_viewer --camera_id 2  # /camera2/image_raw → cam2.jpg

# Specify custom output directory
ros2 run cv_ros_nodes camera_viewer --camera_id 0 --output_dir /path/to/images

# View all cameras (run in separate terminals)
ros2 run cv_ros_nodes camera_viewer --camera_id 0 --output_dir ./camera_images
ros2 run cv_ros_nodes camera_viewer --camera_id 1 --output_dir ./camera_images
ros2 run cv_ros_nodes camera_viewer --camera_id 2 --output_dir ./camera_images
```

**Features:**
- Subscribes to `/camera{N}/image_raw` topics
- Saves each frame as `cam{N}.jpg` in the output directory
- Uses atomic file operations (write to `.tmp.jpg` then rename) to avoid partial files
- Automatically creates output directory if it doesn't exist
- Default output directory: `./camera_images/`

**Output:**
- Each camera saves its latest frame as `cam{N}.jpg` (overwrites previous frame)
- Files are saved atomically to prevent reading partial images

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
      "bbox": [100, 150, 200, 250]
    },
    ...
  ]
}
```

**Key Features:**
- **Latest Value Approach**: Combines the most recent detection from each camera
- **Staleness Filtering**: Automatically excludes cameras that haven't updated recently (configurable threshold)
- **Camera Status**: Each camera has a status: `"active"`, `"stale"`, or `"no_data"`
- **Health Monitoring**: Output includes counts of active/stale/no_data cameras

**Staleness Detection:**
- If a camera's latest detection is older than `staleness_threshold` seconds, it's marked as `"stale"` and excluded from the output
- This helps detect camera failures, processing stalls, or network issues
- Default threshold: 1.0 second (configurable with `--staleness_threshold`)

Each detection includes its source `camera_id` for tracking which camera detected each object.
