# CV-LiDAR Fusion Package

This package implements sensor fusion between computer vision (3 cameras) and LiDAR for buoy detection and tracking. The fusion node populates color information from CV into LiDAR-tracked buoys.

---

## Package Overview

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ Computer Vision │────▶│  Fusion Node     │────▶│    Planning     │
│  (3 Cameras)    │     │                  │     │                 │
└─────────────────┘     └──────────────────┘     └─────────────────┘
      │                          │                         
      │                          │                         
/combined/                /tracked_buoys           
detection_info           (LiDAR input)             
  ~15 Hz                      ↓
                        /fused_buoys
                      (color populated)
                         ~10-30 Hz
```

**Purpose:** Combine CV detections (2D bounding boxes + class_id) with LiDAR tracks (3D positions) to assign buoy colors for autonomous navigation.

---

## Implementation Plan

### Phase 0: Prerequisites ⚠️ CRITICAL

> [!IMPORTANT]
> **These MUST be completed before starting implementation!**

#### 1. Camera Calibration (BLOCKER)

**Status:** ❌ Not completed - **NO calibration files found in codebase**

**What we need:**
- Focal length (fx, fy) in pixels
- Principal point (cx, cy) - image center offset  
- Distortion coefficients (k1, k2, p1, p2, p3)
- Image dimensions (width, height)

**How to obtain: Run ROS2 camera calibration on each camera**

```bash
# For each camera (0, 1, 2):
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.108 \
  image:=/camera{N}/image_raw camera:=/camera{N}

# Save calibration files to: config/camera{N}_calibration.yaml
```

**Checklist:**
- [ ] Camera 0 calibrated → `config/camera0_calibration.yaml`
- [ ] Camera 1 calibrated → `config/camera1_calibration.yaml`
- [ ] Camera 2 calibrated → `config/camera2_calibration.yaml`
- [ ] Verify calibration quality (reprojection error < 0.5 pixels)

**Alternative:** If calibration not possible, can obtain from camera/lens documentation:
- [ ] Focal length from manufacturer specs
- [ ] Sensor size and resolution
- [ ] Calculate fx, fy from: `fx = (image_width / 2) / tan(FOV_horizontal / 2)`

---

#### 2. Camera Mounting Configuration

**Status:** ⚠️ Needs measurement

**What we need:**
- Physical mounting angles of each camera relative to boat forward axis
- TF transforms from `camera{N}` frames to `base_link`

**How to obtain:**
- Measure physical camera angles during installation
- OR extract from TF tree after mounting complete

**Checklist:**
- [ ] Camera 0 yaw angle relative to boat forward: _____ ° (expected: ~-85°)
- [ ] Camera 1 yaw angle: _____ ° (expected: ~0° for center)
- [ ] Camera 2 yaw angle: _____ ° (expected: ~+85°)
- [ ] Verify TF tree has `base_link → camera{0,1,2}` transforms
- [ ] Test TF lookup: `ros2 run tf2_ros tf2_echo base_link camera0`

---

#### 3. Test Data Collection

**Status:** ⚠️ Pending

**What we need:**
- Recorded bag file with both CV and LiDAR detecting buoys simultaneously
- Variety of scenarios (different ranges, colors, multiple buoys)

**How to collect:**
```bash
# Record test data with all necessary topics
ros2 bag record \
  /combined/detection_info \
  /tracked_buoys \
  /tf \
  /tf_static \
  -o fusion_test_$(date +%Y%m%d_%H%M%S)
```

**Checklist:**
- [ ] Test bag recorded with buoys in view
- [ ] Verify both CV and LiDAR detecting same buoys
- [ ] Check bag contains TF data
- [ ] Playback test: `ros2 bag play fusion_test_*.db3`

---

### Phase 1: Basic Node Structure

**Goal:** Create functional ROS2 node that subscribes to inputs and logs data.

**Tasks:**
- [ ] Create `vision_lidar_fusion.py` node
- [ ] Subscribe to `/combined/detection_info` (std_msgs/String - JSON)
- [ ] Subscribe to `/tracked_buoys` (pointcloud_filters/TrackedBuoyArray)
- [ ] Parse JSON from CV messages
- [ ] Log received data to verify subscription works
- [ ] Test with recorded bag file

**Verification:**
```bash
# Terminal 1: Play test bag
ros2 bag play fusion_test_*.db3

# Terminal 2: Run fusion node
ros2 run cv_lidar_fusion vision_lidar_fusion

# Check logs show both CV and LiDAR messages received
```

---

### Phase 2: Coordinate Transform Setup

**Goal:** Set up TF2 and camera intrinsics loading for bearing calculations.

**Tasks:**
- [ ] Initialize TF2 buffer and listener in node
- [ ] Load camera calibration YAML files (from Phase 0)
- [ ] Parse intrinsics: fx, fy, cx, cy for each camera
- [ ] Implement TF lookup: `camera{N} → base_link`
- [ ] Handle TF lookup failures gracefully (log warning, skip frame)
- [ ] Store camera parameters in node state

**Code structure:**
```python
self.camera_intrinsics = {
    0: {'fx': ..., 'fy': ..., 'cx': ..., 'cy': ...},
    1: {'fx': ..., 'fy': ..., 'cx': ..., 'cy': ...},
    2: {'fx': ..., 'fy': ..., 'cx': ..., 'cy': ...},
}
```

**Verification:**
- [ ] Node starts without errors
- [ ] Camera parameters logged on startup
- [ ] TF lookups succeed (log transform values)

---

### Phase 3: Bearing Calculation

**Goal:** Convert CV bbox center pixels to bearing angles in base_link frame.

**Tasks:**
- [ ] Implement `pixel_to_bearing(detection, camera_id)` function
- [ ] Extract bbox center from detection: `(x1+x2)/2, (y1+y2)/2`
- [ ] Convert global frame coords to per-camera coords
- [ ] Use camera intrinsics to get normalized camera coordinates
- [ ] Compute bearing in camera frame: `atan(x_norm)`
- [ ] Transform bearing to base_link using TF2
- [ ] Validate against expected values (sanity checks)

**Bearing calculation formula:**
```python
# 1. Global to local camera coords
frame_width = global_frame_width / 3
bbox_center_u_local = bbox_center_u_global - camera_id * frame_width

# 2. Normalize using intrinsics
x_norm = (bbox_center_u_local - cx) / fx

# 3. Bearing in camera frame
bearing_camera = atan(x_norm)

# 4. Transform to base_link (add camera yaw from TF)
bearing_base_link = bearing_camera + camera_yaw
```

**Verification:**
- [ ] Test with known buoy positions
- [ ] Compare CV bearing to LiDAR bearing (should be similar)
- [ ] Log bearing differences for matched buoys
- [ ] Expected accuracy: within ±5-10° initially

---

### Phase 4: Simple Matching (Prototype)

**Goal:** Match CV detections to LiDAR tracks using bearing similarity.

**Tasks:**
- [ ] Implement `match_vision_to_lidar()` function
- [ ] For each LiDAR track, find closest CV detection by bearing
- [ ] Use bearing threshold: `|bearing_cv - bearing_lidar| < 0.15 rad` (~8.6°)
- [ ] Simple greedy 1-to-1 matching (no Hungarian algorithm yet)
- [ ] Map class_id to color: `0→black, 1-2→green, 3-4→red, 5→yellow`
- [ ] Log matches for debugging

**Class ID to color mapping:**
```python
self.class_to_color = {
    0: "black",
    1: "green",
    2: "green", 
    3: "red",
    4: "red",
    5: "yellow",
    # 6, 7, 8 are non-buoy objects (cross, dock, triangle)
}
```

**Verification:**
- [ ] Playback test bag
- [ ] Check log output shows matches
- [ ] Manually verify correct color assignments (visual inspection)

---

### Phase 5: Publishing Fused Tracks

**Goal:** Publish `/fused_buoys` topic with color information populated.

**Tasks:**
- [ ] Create publisher: `/fused_buoys` (TrackedBuoyArray)
- [ ] Copy LiDAR track data (id, range, bearing, x, y, z_mean, etc.)
- [ ] Update `color` field for matched tracks
- [ ] Keep `color = "unknown"` for unmatched tracks
- [ ] Copy timestamps from LiDAR message header
- [ ] Publish at LiDAR rate (on every `/tracked_buoys` message)

**Message structure (TrackedBuoy):**
```python
fused_track.id = lidar_track.id
fused_track.range = lidar_track.range
fused_track.bearing = lidar_track.bearing
fused_track.x = lidar_track.x
fused_track.y = lidar_track.y
fused_track.z_mean = lidar_track.z_mean
fused_track.color = matched_color  # ← Populated from CV
fused_track.confidence = lidar_track.confidence
fused_track.observation_count = lidar_track.observation_count
# ... timestamps
```

**Verification:**
```bash
# Check topic is publishing
ros2 topic hz /fused_buoys
ros2 topic echo /fused_buoys

# Verify colors are populated (not all "unknown")
```

---

### Phase 6: Synchronization & Buffering

**Goal:** Handle asynchronous CV/LiDAR timing with buffering strategy.

> [!NOTE]
> **LiDAR Update Rate:** `lidar_range_filter` uses 1.0s accumulation window by default, publishes on every incoming cloud. Buoy tracker publishes on every detection (~variable rate). CV publishes at ~15 Hz.

**Synchronization Strategy:**

Since LiDAR tracker uses **exponential smoothing** (alpha=0.7) and requires minimum 3 observations before publishing, and `lidar_range_filter` accumulates points over time, we can use simpler synchronization:

**Option A: Match on LiDAR callback (Recommended)**
- Store latest CV detections in `self.latest_cv_detections`
- On each `/tracked_buoys` message, match against most recent CV data
- No complex buffering needed - CV updates faster than LiDAR

**Option B: Time-based buffering (If needed)**
- Buffer CV detections with timestamps (last 500ms)
- Remove stale detections before matching
- Match LiDAR tracks against buffered CV detections

**Tasks:**
- [ ] Decide on synchronization approach (A vs B)
- [ ] If Option A: Store latest CV in `self.latest_cv_detections`
- [ ] If Option B: Implement time-based CV buffer with staleness check
- [ ] Log timestamp differences when matching
- [ ] Handle case where CV data is stale (>500ms old)

**Code example (Option A):**
```python
def cv_callback(self, msg):
    data = json.loads(msg.data)
    self.latest_cv_detections = data.get('detections', [])
    self.latest_cv_timestamp = msg.header.stamp  # If available
    
def lidar_callback(self, msg):
    # Match against self.latest_cv_detections
    self.process_fusion(msg)
```

**Verification:**
- [ ] Check timestamp differences in logs
- [ ] Verify matches even with async timing
- [ ] Test with delayed CV data (simulate lag)

---

### Phase 7: Color Confidence Voting

**Goal:** Handle conflicting color assignments with confidence-weighted voting.

**Strategy:** Use confidence-weighted voting to resolve conflicts.

**Tasks:**
- [ ] Maintain color history per track: `track_id → [(color, confidence, timestamp)]`
- [ ] Store last N color observations (N=5)
- [ ] When updating color, add to history with CV confidence score
- [ ] Compute weighted vote: most common color weighted by confidence
- [ ] Only update track color if vote confidence > threshold

**Code structure:**
```python
self.color_history = {}  # {track_id: [(color, confidence, timestamp)]}

def update_track_color(self, track_id, new_color, confidence):
    if track_id not in self.color_history:
        self.color_history[track_id] = []
    
    # Add new observation
    self.color_history[track_id].append((new_color, confidence))
    
    # Keep last 5 observations
    self.color_history[track_id] = self.color_history[track_id][-5:]
    
    # Weighted voting
    color_votes = {}
    for color, conf in self.color_history[track_id]:
        color_votes[color] = color_votes.get(color, 0) + conf
    
    # Return color with highest weighted vote
    return max(color_votes, key=color_votes.get)
```

**Verification:**
- [ ] Test with conflicting color detections
- [ ] Verify stable color assignment over time
- [ ] Check color doesn't flip-flop between frames

---

### Phase 8: Edge Case Handling

**Goal:** Handle edge cases robustly.

**Edge cases to handle:**

| Scenario | Action |
|----------|--------|
| No CV detections available | Keep all tracks as `color="unknown"` |
| No LiDAR tracks | Publish empty array (don't create CV-only tracks) |
| Multiple CV detections match same LiDAR track | Take highest confidence CV detection |
| One CV detection matches multiple LiDAR tracks | Match to closest bearing |
| TF lookup fails | Log warning, skip matching for that camera/frame |
| Camera failure (from CV `num_stale_cameras`) | Ignore detections from failed cameras |
| Multi-camera overlap (same buoy in 2 cameras) | Rely on bearing scoring - correct camera wins |

**Tasks:**
- [ ] Implement `None` checks for CV/LiDAR data
- [ ] Handle empty detection arrays
- [ ] Log warnings for TF failures (don't crash)
- [ ] Check CV message `num_stale_cameras` field
- [ ] Add try-except around critical operations

**Verification:**
- [ ] Test with no CV data (disconnect cameras)
- [ ] Test with no LiDAR data
- [ ] Test with stale/old messages
- [ ] Verify node doesn't crash on edge cases

---

### Phase 9: Advanced Matching (Optional Enhancement)

**Goal:** Improve matching accuracy with multi-factor scoring.

**Enhancements:**
- [ ] Implement range estimation from bbox size
- [ ] Add range validation to matching score
- [ ] Use Hungarian algorithm for optimal assignment (scipy)
- [ ] Handle multi-camera overlap with de-duplication

**Scoring function:**
```python
def compute_match_score(vision_det, lidar_track):
    # Bearing similarity
    bearing_diff = abs(vision_bearing - lidar_track.bearing)
    bearing_score = exp(-bearing_diff / 0.1)
    
    # Range validation (optional, if implemented)
    estimated_range = estimate_range_from_bbox(vision_det)
    range_diff = abs(estimated_range - lidar_track.range)
    range_score = exp(-range_diff / 2.0)
    
    # Confidence weighting
    conf_score = vision_det['score'] * lidar_track.confidence
    
    # Weighted combination
    return 0.6*bearing_score + 0.3*range_score + 0.1*conf_score
```

**Verification:**
- [ ] Compare matching accuracy vs. simple bearing-only
- [ ] Measure improvement in color assignment rate
- [ ] Tune weights based on real-world performance

---

## Pipeline Architecture Details

### LiDAR Pipeline

```
/unilidar/cloud (raw point cloud)
    ↓
lidar_range_filter (z-filter + accumulation)
  - accumulation_window: 1.0s (configurable)
  - publishes accumulated clouds
    ↓
buoy_detector (DBSCAN clustering + RANSAC water removal)
  - detects buoy clusters
    ↓
/buoy_detections (BuoyDetectionArray)
    ↓
buoy_tracker (data association + smoothing)
  - exponential smoothing (alpha=0.7)
  - min 3 observations before publishing
  - publishes on every detection callback
    ↓
/tracked_buoys (TrackedBuoyArray) ← FUSION INPUT
```

**Key characteristics:**
- LiDAR tracker uses **exponential smoothing** for stable position estimates
- Tracks require **minimum 3 observations** before being published
- Publishing frequency: **variable**, depends on detection rate (~10-30 Hz typical)
- Tracks have **persistent IDs** maintained across frames

### CV Pipeline

```
3 Cameras (camera0, camera1, camera2)
    ↓
YOLO inference per camera (~15 Hz)
    ↓
vision_combiner (merges 3 camera outputs)
    ↓
/combined/detection_info (JSON) ← FUSION INPUT
  - 15 Hz typical
  - contains: bbox, class_id, camera_id, score
  - global frame coordinates (1920×480 for 3×640×480 cameras)
```

**Key characteristics:**
- CV runs at **~15 Hz** (faster than LiDAR tracking)
- Detections are **instantaneous** (single frame)
- No temporal filtering - pure per-frame detections
- Multi-camera overlap: **15° between adjacent cameras**

---

## Configuration Files

### Camera Calibration Format

Expected YAML format (standard ROS2 camera_info):

```yaml
# config/camera0_calibration.yaml
image_width: 640
image_height: 480
camera_name: camera0
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
```

### Launch File Configuration

```python
# launch/fusion.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cv_lidar_fusion',
            executable='vision_lidar_fusion',
            name='fusion_node',
            parameters=[{
                'camera0_calibration': 'config/camera0_calibration.yaml',
                'camera1_calibration': 'config/camera1_calibration.yaml',
                'camera2_calibration': 'config/camera2_calibration.yaml',
                'bearing_threshold': 0.15,  # radians (~8.6°)
                'color_vote_window': 5,  # frames
            }],
            output='screen'
        )
    ])
```

---

## Testing & Validation

### Unit Tests

- [ ] Test `class_to_color` mapping
- [ ] Test `pixel_to_bearing()` with known inputs
- [ ] Test matching with mock CV/LiDAR data
- [ ] Test color voting logic

### Integration Tests

```bash
# 1. Launch full pipeline with test bag
ros2 bag play fusion_test_*.db3

# 2. Run fusion node
ros2 run cv_lidar_fusion vision_lidar_fusion

# 3. Monitor output
ros2 topic hz /fused_buoys
ros2 topic echo /fused_buoys | grep color

# 4. Check color assignment rate
# Expected: >80% of tracks should have colors (not "unknown")
```

### Performance Benchmarks

- [ ] Measure fusion node CPU usage (target: <10%)
- [ ] Measure latency: CV detection → fused output (target: <100ms)
- [ ] Check memory usage with long-running test
- [ ] Profile matching performance (target: <10ms per frame)

---

## Known Limitations & Future Work

### Current Limitations

1. **Bearing-only matching** - No range validation (yet)
2. **Simple greedy matching** - Not optimal assignment
3. **No occlusion handling** - Assumes all buoys visible
4. **Static camera calibration** - No online recalibration

### Future Enhancements

- [ ] Implement range estimation from bbox size
- [ ] Add Hungarian algorithm for optimal matching
- [ ] Handle multi-camera overlap de-duplication
- [ ] Add Kalman filter for smoother color transitions
- [ ] Support dynamic camera calibration updates
- [ ] Add diagnostics publishing (match quality metrics)

---

## Troubleshooting

### Common Issues

**Issue:** No colors being assigned (all "unknown")
- Check CV topic is publishing: `ros2 topic hz /combined/detection_info`
- Verify bearing calculations are working (check logs)
- Check bearing threshold isn't too strict (try increasing to 0.2)

**Issue:** Wrong colors assigned
- Verify camera calibration is correct
- Check class_id mapping matches your training data
- Increase color voting window for stability

**Issue:** TF lookup failures
- Verify TF tree: `ros2 run tf2_tools view_frames`
- Check camera frames are published: `ros2 topic echo /tf`
- Ensure static transforms are defined

**Issue:** Performance problems
- Profile with: `ros2 run cv_lidar_fusion vision_lidar_fusion --ros-args --log-level debug`
- Check if matching is too slow (add timing logs)
- Consider reducing CV detections processing (filter by confidence)

---

## References

- **Original Fusion Guide:** `autonomy-ws-25-26/computer_vision/FUSION_NODE_GUIDE.md`
- **Class Mapping:** `autonomy-ws-25-26/computer_vision/cv_scripts/class_mapping.yaml`
- **TrackedBuoy Message:** `autonomy-ws-25-26/mapping/src/pointcloud_filters/msg/TrackedBuoy.msg`
- **Buoy Tracker:** `autonomy-ws-25-26/mapping/src/pointcloud_filters/pointcloud_filters/buoy_tracker.py`
- **LiDAR Filter:** `autonomy-ws-25-26/mapping/src/pointcloud_filters/pointcloud_filters/lidar_range_filter.py`

---

## Contributors

- Design Team: FUSION_NODE_GUIDE.md
- Implementation: [Your team]

---

*Last Updated: January 26, 2026*
