# Computer Vision + LiDAR Fusion Node Guide

This document provides all the information needed to build a fusion node that combines computer vision detections with LiDAR-based buoy tracking.

---

## Table of Contents

1. [Pipeline Overview](#pipeline-overview)
2. [Computer Vision Output](#computer-vision-output)
3. [LiDAR/Mapping System](#lidarmapping-system)
4. [Fusion Node Requirements](#fusion-node-requirements)
5. [Message Formats](#message-formats)
6. [Coordinate Systems & Transforms](#coordinate-systems--transforms)
7. [Data Association Strategy](#data-association-strategy)
8. [Implementation Checklist](#implementation-checklist)

---

## Pipeline Overview

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ Computer Vision │────▶│  Fusion Node     │────▶│    Mapping      │
│  (3 Cameras)    │     │  (YOU BUILD)     │     │  (LiDAR Track)  │
└─────────────────┘     └──────────────────┘     └─────────────────┘
     │                          │                         │
     │                          │                         │
/combined/              /fused_buoys            /tracked_buoys
detection_info          (or update              (with color)
                        /tracked_buoys)         ──────▶ Planning
```

**Current State:**
- ✅ Computer Vision: Fully implemented, publishes `/combined/detection_info`
- ✅ LiDAR/Mapping: Fully implemented, publishes `/tracked_buoys` (color = "unknown")
- ❌ Fusion Node: **YOU NEED TO BUILD THIS**

---

## Computer Vision Output

### Topic: `/combined/detection_info`

**Type**: `std_msgs/String` (JSON format)

**Publisher**: `vision_combiner` node

**Frequency**: ~30 Hz

### Message Structure

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
      "height": 100.0,
      "bbox": [100, 150, 200, 250]
    }
  ]
}
```

### Detection Fields

Each detection in the `detections` array contains:

| Field | Type | Description |
|-------|------|-------------|
| `camera_id` | int | Source camera (0, 1, or 2) |
| `class_id` | int | Object class identifier (see class mapping below) |
| `score` | float | Confidence score (0.0-1.0) |
| `x1, y1` | float | Top-left bbox corner (pixels) |
| `x2, y2` | float | Bottom-right bbox corner (pixels) |
| `width, height` | float | Bounding box dimensions (pixels) |
| `bbox` | array | `[x1, y1, x2, y2]` for convenience |

### Class ID Mapping

Located in: `cv_scripts/class_mapping.yaml`

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

**Color Mapping for Fusion:**
- `class_id` 0 → color = `"black"`
- `class_id` 1, 2 → color = `"green"`
- `class_id` 3, 4 → color = `"red"`
- `class_id` 5 → color = `"yellow"`
- `class_id` 6, 7, 8 → Not buoys (shapes/infrastructure)

### Camera Configuration

**Physical Setup:**
- 3 cameras mounted side-by-side
- Field of view per camera: 85° (horizontal)
- Overlap between adjacent cameras: 15°
- Total horizontal coverage: ~245°
- Camera frames: `camera0`, `camera1`, `camera2`
- Image resolution: 640x480 (preprocessed from 1920x1200)

**Camera Positions:**
- Camera 0: Left camera
- Camera 1: Center camera
- Camera 2: Right camera

---

## LiDAR/Mapping System

### Topic: `/tracked_buoys`

**Type**: `pointcloud_filters/TrackedBuoyArray`

**Publisher**: `buoy_tracker` node (in mapping package)

**Frequency**: ~10-30 Hz (depends on LiDAR detection rate)

**Location**: `/home/lorenzo/autonomy-ws-25-26/mapping/src/pointcloud_filters/`

### Message Structure

```python
# TrackedBuoyArray.msg
std_msgs/Header header
TrackedBuoy[] buoys

# TrackedBuoy.msg
uint32 id                           # Unique persistent ID (starts at 0, increments)
float32 range                       # Distance from sensor (meters)
float32 bearing                     # Angle from sensor (radians, -π to +π)
float32 x                           # Cartesian x position (meters, in base_link)
float32 y                           # Cartesian y position (meters, in base_link)
float32 z_mean                      # Average z height (meters)
string color                        # Color from vision: "red", "green", "yellow", "unknown"
float32 confidence                  # Detection confidence (0.0 to 1.0)
uint32 observation_count            # Number of times this buoy has been detected
builtin_interfaces/Time first_seen  # Timestamp when first detected
builtin_interfaces/Time last_seen   # Timestamp of most recent detection
```

### LiDAR Pipeline

```
/unilidar/cloud (PointCloud2)
    ↓
lidar_range_filter
    ↓
/points_filtered (PointCloud2)
    ↓
buoy_detector (DBSCAN clustering)
    ↓
/buoy_detections (BuoyDetectionArray)
    ↓
buoy_tracker (data association, smoothing)
    ↓
/tracked_buoys (TrackedBuoyArray) ← YOU SUBSCRIBE TO THIS
```

### Coordinate System

- **Frame**: `base_link` (or `unilidar_lidar` if TF transform disabled)
- **Origin**: Boat center
- **X-axis**: Forward (bow direction)
- **Y-axis**: Left (port side)
- **Z-axis**: Up
- **Bearing**: Angle from +X axis, positive = counter-clockwise (CCW), range [-π, +π]

### LiDAR Detection Characteristics

- **Range**: Typically 0.5m - 30m (configurable)
- **Bearing**: Full 360° coverage (horizontal plane)
- **Update Rate**: ~10-30 Hz (depends on LiDAR scan rate)
- **Tracking**: Persistent IDs, exponential smoothing, observation filtering

---

## Fusion Node Requirements

### Purpose

Combine computer vision detections (2D pixel coordinates + class_id) with LiDAR tracks (3D positions) to:
1. **Match** vision detections to LiDAR tracks
2. **Populate** color information in tracked buoys
3. **Validate** detections using both sensors
4. **Enhance** tracking with vision confidence scores

### Inputs (Subscriptions)

1. **`/combined/detection_info`** (std_msgs/String)
   - JSON string with vision detections
   - Contains: bbox, class_id, camera_id, score

2. **`/tracked_buoys`** (pointcloud_filters/TrackedBuoyArray)
   - LiDAR-tracked buoys with range, bearing, x, y
   - Currently has `color = "unknown"`

### Outputs (Publications)

**Option 1: Update Existing Topic**
- Publish to `/tracked_buoys` with color information populated
- Overwrites the LiDAR-only tracker output

**Option 2: New Fused Topic**
- Publish to `/fused_buoys` (TrackedBuoyArray)
- Keeps original `/tracked_buoys` intact
- Planning subscribes to `/fused_buoys`

**Recommended**: Option 2 (new topic) for safety and debugging

### Node Name

Suggested: `vision_lidar_fusion` or `cv_lidar_fusion`

---

## Message Formats

### Input: Vision Detection (from JSON)

```python
{
    "camera_id": 0,        # 0, 1, or 2
    "class_id": 3,         # 0-8 (see class mapping)
    "score": 0.85,         # 0.0-1.0
    "x1": 100.0,           # Top-left x (pixels)
    "y1": 150.0,           # Top-left y (pixels)
    "x2": 200.0,           # Bottom-right x (pixels)
    "y2": 250.0,           # Bottom-right y (pixels)
    "width": 100.0,        # bbox width (pixels)
    "height": 100.0,       # bbox height (pixels)
    "bbox": [100, 150, 200, 250]
}
```

### Input: LiDAR Track

```python
# TrackedBuoy message
id = 5
range = 12.5              # meters
bearing = 0.523           # radians (~30°)
x = 10.8                  # meters (in base_link)
y = 6.2                   # meters (in base_link)
z_mean = 0.3              # meters
color = "unknown"         # ← YOU NEED TO POPULATE THIS
confidence = 0.75
observation_count = 15
```

### Output: Fused Track

```python
# TrackedBuoy message (same structure, with color populated)
id = 5                    # Same ID from LiDAR tracker
range = 12.5              # From LiDAR
bearing = 0.523           # From LiDAR
x = 10.8                  # From LiDAR
y = 6.2                   # From LiDAR
z_mean = 0.3              # From LiDAR
color = "red"             # ← POPULATED FROM VISION class_id
confidence = 0.85         # Could combine vision + LiDAR confidence
observation_count = 15    # From LiDAR
```

---

## Coordinate Systems & Transforms

### Vision Coordinate System

**Camera Frames:**
- `camera0`, `camera1`, `camera2`
- Image coordinates: (0, 0) = top-left, (640, 480) = bottom-right
- Bounding boxes: pixel coordinates in image frame

**To Convert to 3D:**
1. Get camera-to-base_link transform (TF2)
2. Project bbox center to 3D ray using camera intrinsics
3. Intersect ray with estimated range from LiDAR

### LiDAR Coordinate System

**Frame:** `base_link`
- Origin: Boat center
- X: Forward (bow)
- Y: Left (port)
- Z: Up
- Bearing: Angle from +X axis, CCW positive

### Transform Requirements

You'll need to:
1. **Lookup TF transforms**: `camera{N} → base_link`
2. **Camera intrinsics**: Focal length, principal point (for projection)
3. **Project bbox center**: Convert pixel (u, v) → 3D ray → bearing estimate

**TF2 Usage:**
```python
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# In node __init__:
self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
self.tf_listener = TransformListener(self.tf_buffer, self)

# In callback:
transform = self.tf_buffer.lookup_transform(
    'base_link',
    f'camera{camera_id}',
    msg.header.stamp,
    timeout=Duration(seconds=0.1)
)
```

---

## Data Association Strategy

### Challenge

Match vision detections (2D pixels, camera frame) to LiDAR tracks (3D positions, base_link frame).

### Approach 1: Bearing-Based Matching (Recommended)

1. **Project vision bbox center to bearing:**
   - Get bbox center: `(u, v) = ((x1+x2)/2, (y1+y2)/2)`
   - Convert pixel to bearing using camera intrinsics
   - Account for camera FOV and position

2. **Match by bearing similarity:**
   - For each vision detection, find closest LiDAR track by bearing
   - Threshold: `|bearing_vision - bearing_lidar| < threshold` (e.g., 0.1 rad ≈ 5.7°)

3. **Validate with range:**
   - Estimate range from bbox size (larger bbox = closer object)
   - Check if estimated range matches LiDAR range (within tolerance)

### Approach 2: 3D Projection Matching

1. **Project vision bbox to 3D:**
   - Use camera intrinsics + extrinsics (TF2)
   - Project bbox center to 3D ray
   - Intersect with LiDAR range estimate

2. **Match by 3D distance:**
   - Compute 3D position from vision projection
   - Match to closest LiDAR track by Euclidean distance

### Approach 3: Hybrid (Bearing + Range + Confidence)

1. **Compute matching score for each (vision, lidar) pair:**
   ```python
   score = (
       bearing_similarity_weight * exp(-|bearing_diff|) +
       range_similarity_weight * exp(-|range_diff| / range_scale) +
       confidence_weight * (vision_confidence * lidar_confidence)
   )
   ```

2. **Hungarian algorithm or greedy matching:**
   - Assign each vision detection to best-matching LiDAR track
   - One-to-one matching (each vision detection → at most one LiDAR track)

### Recommended: Approach 1 (Bearing-Based)

**Simplest and most robust for initial implementation:**

```python
def match_vision_to_lidar(vision_detection, lidar_tracks, camera_id):
    """
    Match a vision detection to a LiDAR track.
    
    Returns: (matched_track_id, match_confidence) or (None, 0.0)
    """
    # 1. Compute vision bearing from bbox center
    bbox_center_u = (vision_detection['x1'] + vision_detection['x2']) / 2.0
    bbox_center_v = (vision_detection['y1'] + vision_detection['y2']) / 2.0
    
    # Convert pixel to bearing (requires camera intrinsics)
    vision_bearing = pixel_to_bearing(bbox_center_u, bbox_center_v, camera_id)
    
    # 2. Find closest LiDAR track by bearing
    best_match = None
    best_bearing_diff = float('inf')
    bearing_threshold = 0.15  # ~8.6 degrees
    
    for track in lidar_tracks:
        bearing_diff = abs(vision_bearing - track.bearing)
        if bearing_diff < bearing_threshold and bearing_diff < best_bearing_diff:
            best_bearing_diff = bearing_diff
            best_match = track
    
    return best_match
```

---

## Implementation Checklist

### Phase 1: Basic Node Structure

- [ ] Create ROS2 node: `vision_lidar_fusion.py`
- [ ] Subscribe to `/combined/detection_info` (std_msgs/String)
- [ ] Subscribe to `/tracked_buoys` (TrackedBuoyArray)
- [ ] Parse JSON from vision detections
- [ ] Log received messages (verify data flow)

### Phase 2: Coordinate Transform

- [ ] Initialize TF2 buffer and listener
- [ ] Lookup transform: `camera{N} → base_link`
- [ ] Test transform lookup (handle failures gracefully)
- [ ] Get camera intrinsics (focal length, principal point)
- [ ] Implement pixel-to-bearing conversion function

### Phase 3: Data Association

- [ ] Implement bearing-based matching
- [ ] Match vision detections to LiDAR tracks
- [ ] Handle multiple vision detections per LiDAR track
- [ ] Handle unmatched detections (vision-only or LiDAR-only)

### Phase 4: Color Population

- [ ] Map class_id to color string:
  - `0 → "black"`, `1,2 → "green"`, `3,4 → "red"`, `5 → "yellow"`
- [ ] Update TrackedBuoy.color field
- [ ] Handle multiple vision detections (take highest confidence or most recent)

### Phase 5: Publishing

- [ ] Create publisher: `/fused_buoys` (TrackedBuoyArray)
- [ ] Copy LiDAR tracks to output message
- [ ] Update color field for matched tracks
- [ ] Publish at appropriate rate (match LiDAR or vision rate)

### Phase 6: Testing & Validation

- [ ] Test with real data (recorded bag or live)
- [ ] Verify color assignment is correct
- [ ] Check matching accuracy (bearing alignment)
- [ ] Handle edge cases:
  - No vision detections
  - No LiDAR tracks
  - TF lookup failures
  - Camera failures

---

## Example Code Structure

```python
#!/usr/bin/env python3
"""
Vision-LiDAR Fusion Node

Combines computer vision detections with LiDAR tracks to populate color information.
"""

import json
import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.duration import Duration

from std_msgs.msg import String
from pointcloud_filters.msg import TrackedBuoyArray, TrackedBuoy
from tf2_ros import Buffer, TransformListener


class VisionLidarFusionNode(Node):
    def __init__(self):
        super().__init__('vision_lidar_fusion')
        
        # TF2 for coordinate transforms
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.vision_sub = self.create_subscription(
            String,
            '/combined/detection_info',
            self.vision_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            TrackedBuoyArray,
            '/tracked_buoys',
            self.lidar_callback,
            10
        )
        
        # Publisher
        self.fused_pub = self.create_publisher(
            TrackedBuoyArray,
            '/fused_buoys',
            10
        )
        
        # State
        self.latest_lidar_tracks = None
        self.latest_vision_detections = None
        
        # Class ID to color mapping
        self.class_to_color = {
            0: "black",
            1: "green",
            2: "green",
            3: "red",
            4: "red",
            5: "yellow"
        }
        
        self.get_logger().info('Vision-LiDAR fusion node started')
    
    def vision_callback(self, msg: String):
        """Handle vision detections."""
        try:
            data = json.loads(msg.data)
            self.latest_vision_detections = data.get('detections', [])
            self.get_logger().debug(f'Received {len(self.latest_vision_detections)} vision detections')
            self.process_fusion()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse vision JSON: {e}')
    
    def lidar_callback(self, msg: TrackedBuoyArray):
        """Handle LiDAR tracks."""
        self.latest_lidar_tracks = msg
        self.get_logger().debug(f'Received {len(msg.buoys)} LiDAR tracks')
        self.process_fusion()
    
    def process_fusion(self):
        """Main fusion logic: match vision to LiDAR and populate colors."""
        if self.latest_lidar_tracks is None or self.latest_vision_detections is None:
            return
        
        # Create output message (copy LiDAR tracks)
        fused_msg = TrackedBuoyArray()
        fused_msg.header = self.latest_lidar_tracks.header
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Match vision detections to LiDAR tracks
        for track in self.latest_lidar_tracks.buoys:
            # Copy track
            fused_track = TrackedBuoy()
            fused_track.id = track.id
            fused_track.range = track.range
            fused_track.bearing = track.bearing
            fused_track.x = track.x
            fused_track.y = track.y
            fused_track.z_mean = track.z_mean
            fused_track.confidence = track.confidence
            fused_track.observation_count = track.observation_count
            fused_track.first_seen = track.first_seen
            fused_track.last_seen = track.last_seen
            
            # Try to match with vision detection
            matched_detection = self.match_vision_to_track(track)
            if matched_detection:
                # Populate color from vision
                class_id = matched_detection.get('class_id')
                fused_track.color = self.class_to_color.get(class_id, "unknown")
                self.get_logger().info(
                    f'Matched track {track.id} to vision detection: '
                    f'class_id={class_id}, color={fused_track.color}'
                )
            else:
                # Keep existing color (or "unknown")
                fused_track.color = track.color if track.color != "" else "unknown"
            
            fused_msg.buoys.append(fused_track)
        
        # Publish fused tracks
        self.fused_pub.publish(fused_msg)
        self.get_logger().info(f'Published {len(fused_msg.buoys)} fused tracks')
    
    def match_vision_to_track(self, track: TrackedBuoy) -> Optional[Dict]:
        """
        Match a LiDAR track to a vision detection based on bearing.
        
        Returns: Matched vision detection dict or None
        """
        best_match = None
        best_bearing_diff = float('inf')
        bearing_threshold = 0.15  # ~8.6 degrees
        
        for detection in self.latest_vision_detections:
            camera_id = detection.get('camera_id')
            
            # Compute vision bearing from bbox center
            # TODO: Implement pixel_to_bearing() using camera intrinsics
            vision_bearing = self.pixel_to_bearing(detection, camera_id)
            if vision_bearing is None:
                continue
            
            # Compare with LiDAR bearing
            bearing_diff = abs(vision_bearing - track.bearing)
            if bearing_diff < bearing_threshold and bearing_diff < best_bearing_diff:
                best_bearing_diff = bearing_diff
                best_match = detection
        
        return best_match
    
    def pixel_to_bearing(self, detection: Dict, camera_id: int) -> Optional[float]:
        """
        Convert vision bbox center pixel to bearing angle in base_link frame.
        
        TODO: Implement using:
        1. Camera intrinsics (focal length, principal point)
        2. TF transform: camera{N} → base_link
        3. Camera FOV and mounting angle
        """
        # Placeholder: simple approximation
        # In reality, you need camera calibration and TF transforms
        bbox_center_u = (detection['x1'] + detection['x2']) / 2.0
        image_width = 640  # Preprocessed image width
        
        # Simple approximation: assume camera FOV = 85°, centered
        # This is a placeholder - implement proper projection!
        normalized_u = (bbox_center_u - image_width / 2.0) / (image_width / 2.0)
        fov_rad = math.radians(85.0)
        vision_bearing_local = normalized_u * (fov_rad / 2.0)
        
        # Account for camera mounting angle (camera 0 = left, camera 1 = center, camera 2 = right)
        # TODO: Get actual camera mounting angles from TF or config
        camera_offsets = {
            0: math.radians(-85),  # Left camera
            1: math.radians(0),      # Center camera
            2: math.radians(85)     # Right camera
        }
        
        vision_bearing = vision_bearing_local + camera_offsets.get(camera_id, 0.0)
        
        return vision_bearing


def main(args=None):
    rclpy.init(args=args)
    node = VisionLidarFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Key Implementation Notes

### 1. Camera Intrinsics

You'll need camera calibration parameters:
- Focal length (fx, fy)
- Principal point (cx, cy)
- Distortion coefficients (optional)

**Where to get them:**
- Camera calibration files (YAML format, typically in `config/` or `calibration/`)
- Or use approximate values based on camera specs

### 2. Camera Extrinsics

Camera mounting positions and orientations:
- Get from TF2 transforms: `camera{N} → base_link`
- Or hardcode if cameras are fixed relative to boat

### 3. Bearing Calculation

**Proper method:**
1. Get bbox center pixel: `(u, v)`
2. Undistort (if needed)
3. Convert to normalized camera coordinates: `(x_norm, y_norm)`
4. Compute bearing in camera frame: `atan2(x_norm, 1.0)` (assuming forward = +Z)
5. Transform to base_link frame using TF2

**Simplified method (for testing):**
- Assume linear mapping: pixel position → bearing
- Account for camera FOV and mounting angle

### 4. Matching Thresholds

**Bearing threshold:**
- Start with `0.15 rad` (~8.6°)
- Tune based on camera FOV and mounting accuracy

**Range validation (optional):**
- Estimate range from bbox size
- Validate against LiDAR range (within ±2m tolerance)

### 5. Handling Edge Cases

- **No vision detections**: Keep LiDAR tracks with `color = "unknown"`
- **No LiDAR tracks**: Don't publish (or publish empty array)
- **Multiple vision detections match same track**: Take highest confidence
- **TF lookup fails**: Skip matching for that camera, log warning
- **Camera not in view**: Skip detections from that camera

---

## Testing

### 1. Unit Testing

Test individual functions:
- `pixel_to_bearing()` with known inputs
- `match_vision_to_track()` with mock data
- Class ID to color mapping

### 2. Integration Testing

```bash
# Terminal 1: Launch computer vision
cd ~/autonomy-ws-25-26/computer_vision
source install/setup.bash
ros2 launch cv_ros_nodes launch_cv.py

# Terminal 2: Launch LiDAR pipeline
cd ~/autonomy-ws-25-26/mapping
source install/setup.bash
ros2 launch pointcloud_filters lidar_range_filter.launch.py
ros2 launch pointcloud_filters buoy_detector.launch.py
ros2 run pointcloud_filters buoy_tracker

# Terminal 3: Launch fusion node
ros2 run <your_package> vision_lidar_fusion

# Terminal 4: Monitor topics
ros2 topic echo /fused_buoys
ros2 topic hz /fused_buoys
```

### 3. Validation

- Check that colors are populated correctly
- Verify bearing alignment (vision vs LiDAR)
- Monitor matching rate (how many tracks get colors)
- Check for false matches

---

## Next Steps

1. **Implement basic node structure** (Phase 1)
2. **Get camera calibration data** (focal length, principal point)
3. **Implement pixel-to-bearing conversion** (Phase 2)
4. **Implement matching logic** (Phase 3)
5. **Test with recorded data** (bag file if available)
6. **Tune thresholds** based on real-world performance
7. **Integrate with planning system** (ensure planning subscribes to `/fused_buoys`)

---

## References

- **Computer Vision README**: `/home/lorenzo/autonomy-ws-25-26/computer_vision/README.md`
- **Mapping README**: `/home/lorenzo/autonomy-ws-25-26/mapping/README.md`
- **Class Mapping**: `/home/lorenzo/autonomy-ws-25-26/computer_vision/cv_scripts/class_mapping.yaml`
- **TrackedBuoy Message**: `/home/lorenzo/autonomy-ws-25-26/mapping/src/pointcloud_filters/msg/TrackedBuoy.msg`

---

*Last Updated: January 2025*
