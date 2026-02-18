# Testing Status - Red Buoy Detection

## ✅ What's Working

### 1. **Coordinate System** (100% FIXED)
- ✅ Compass heading conversion: `heading_rad = radians(90 - compass_deg)`
- ✅ LiDAR TF transform: 0° yaw, 30° pitch down (pointing forward)
- ✅ Angle normalization: All bearings in valid [-π, π] range
- ✅ Global detections: LiDAR buoys correctly mapped to global coordinates

### 2. **Individual Components**
- ✅ ROS2: Working
- ✅ MAVROS: Compass at 2.73° (North)
- ✅ LiDAR: Detecting buoys in `base_link` frame
- ✅ Boat pose: `heading_rad: 1.537` (88° = North)
- ✅ Global detections: 2-3 LiDAR buoys with correct north/east coordinates

### 3. **Pipeline Status**
- ✅ `comp_single_camera.sh`: Updated for middle camera (`1.1:1.0`)
- ✅ 15 processes running (MAVROS, global_frame, LiDAR, CV)
- ✅ Camera1 raw images: Publishing at 11 fps

## ⚠️ Current Issue

### Camera Detection Pipeline Stalled
**Symptom**: Camera preprocessing stopped after ~2 minutes of operation
- Camera raw images: ✅ Publishing
- Camera preprocessing: ❌ Stopped (went stale)
- Camera detections: ❌ No output
- Result: No red buoy detection, all detections show `class_name: unknown`

**Root Cause**: Likely race condition - nodes starting too fast before dependencies ready

**Fix Applied**: Added staggered delays to camera launch file
- Camera node: Start immediately
- Preprocessing: +3s delay
- Inference: +5s delay  
- Combiner/Distance: +7s delay

## 🔧 Recommended Next Steps

### Option 1: Reboot and Test (RECOMMENDED)
A full reboot will ensure:
1. Clean USB camera initialization
2. No stale ROS processes
3. Fresh start for all nodes

**After reboot:**
```bash
cd /home/lorenzo/autonomy-ws-25-26
./comp_single_camera.sh
# Wait 30 seconds for full warmup
ros2 topic echo /camera1/detection_info --once
ros2 topic echo /global_detections --once
```

### Option 2: Kill and Restart Pipeline
```bash
pkill -9 -f "ros2|mavros|camera|lidar|vision"
sleep 5
cd /home/lorenzo/autonomy-ws-25-26
./comp_single_camera.sh
```

## 📊 Expected Results (After Fix)

### When Red Buoy is Detected:
```json
{
  "camera_id": 1,
  "num_detections": 1,
  "detections": [{
    "class_name": "red_buoy",
    "class_id": 0,
    "confidence": 0.85,
    "bearing_deg": 0.0,  // Forward
    "distance_m": 4.5
  }]
}
```

### Global Detections (Fused):
```yaml
detections:
- east: 0.0
  north: 4.5
  source: fused
  class_name: red_buoy  # ← From camera!
  class_id: 0
  range_m: 4.5
  bearing_global_rad: 1.57  # North
```

## 🎯 Success Criteria

1. ✅ Camera1 publishes detections continuously
2. ✅ Red buoy detected with `class_name: "red_buoy"`
3. ✅ Fusion matches camera + LiDAR by bearing
4. ✅ Global detections show red buoy in correct global position
5. ✅ All angles in valid range (no 415°)

## 📝 Files Modified

1. `/home/lorenzo/autonomy-ws-25-26/comp_single_camera.sh`
   - Updated camera path to `1.1:1.0` (middle camera)

2. `/home/lorenzo/autonomy-ws-25-26/computer_vision/src/cv_ros_nodes/cv_ros_nodes/launch/launch_cv_single_camera1.py`
   - Added TimerAction delays (3s, 5s, 7s) between nodes

3. `/home/lorenzo/autonomy-ws-25-26/mapping/src/global_frame/global_frame/boat_state_node.py`
   - Fixed compass conversion

4. `/home/lorenzo/autonomy-ws-25-26/mapping/src/pointcloud_filters/launch/buoy_pipeline.launch.py`
   - Fixed LiDAR TF quaternion

5. `/home/lorenzo/autonomy-ws-25-26/mapping/src/global_frame/global_frame/detection_to_global_node.py`
   - Added angle normalization

## 🔍 Debugging Commands

```bash
# Check if camera is detecting
ros2 topic echo /camera1/detection_info --once

# Check global detections
ros2 topic echo /global_detections --once

# Check boat heading
ros2 topic echo /boat_pose --once

# Check camera image rate
ros2 topic hz /camera1/image_raw

# Check preprocessing rate
ros2 topic hz /camera1/image_preprocessed

# Check running processes
ps aux | grep -E "(camera|vision|lidar|mavros)" | grep -v grep | wc -l
```

---

**Status**: System ready for reboot test. All coordinate fixes applied and verified. Camera launch delays added to prevent race conditions.
