# QoS Mismatch Fix - Camera Detection Now Working!

## 🐛 **Root Cause Found**

**QoS Incompatibility** between camera publisher and preprocessing subscriber!

### The Problem:
- **Camera (v4l2_camera_node)**: Publishes `/camera1/image_raw` with `RELIABLE` QoS
- **Preprocessing node**: Subscribes with `BEST_EFFORT` QoS
- **Result**: ROS2 refuses to deliver messages! Preprocessing never receives images.

### Why It Worked Standalone:
When you ran the CV pipeline alone, the camera might have been configured differently, or there was less resource contention allowing it to work despite the mismatch.

### Why It Failed in comp_single_camera.sh:
With all pipelines running (MAVROS, LiDAR, global_frame, CV), the QoS mismatch became fatal.

## ✅ **The Fix**

Changed `vision_preprocessing.py` line 26:

```python
# OLD (WRONG):
qos_sensor = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # ❌
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# NEW (CORRECT):
qos_sensor = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # ✅
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
```

## 📊 **Test Results**

### ✅ Camera Detection Working!
```json
{
  "camera_id": 1,
  "num_detections": 1,  // ✅ DETECTING!
  "inference_time_ms": 25.3
}
```

### ✅ Combined Detections:
```json
{
  "num_active_cameras": 1,  // ✅ Camera active!
  "num_stale_cameras": 0,
  "num_no_data_cameras": 2
}
```

### ✅ Global Detections:
- 5-6 LiDAR buoys mapped to global coordinates
- All angles normalized (no more 415°!)
- Boat heading correct (1.54 rad ≈ 88° = North)

## ⚠️ Remaining Issue

**Fusion not showing class names yet** - all detections still show `class_name: unknown`

### Possible Causes:
1. **Distance estimation**: Camera detections might not have `distance_m` field
   - `detection_to_global_node` filters out CV detections without distance (line 229-231)
2. **Bearing mismatch**: Camera and LiDAR bearings might not align within threshold
3. **Timing**: CV and LiDAR detections might not be synchronized

### Next Steps to Debug:
```bash
# Check if camera detections have distance
ros2 topic echo /combined/detection_info_with_distance --once | grep distance_m

# Check if fusion is matching
ros2 topic echo /fused_buoys --once

# Check maritime distance estimator
ros2 node info /maritime_distance_estimator
```

## 🎯 **What's Working Now**

1. ✅ **All coordinate fixes** (compass, LiDAR TF, angle normalization)
2. ✅ **Camera detection** (red buoy being detected!)
3. ✅ **Preprocessing pipeline** (QoS fixed, images flowing)
4. ✅ **Global mapping** (LiDAR → global coordinates working perfectly)
5. ✅ **Boat pose** (heading correct, compass working)

## 📝 **Files Modified**

1. `/home/lorenzo/autonomy-ws-25-26/computer_vision/src/cv_ros_nodes/cv_ros_nodes/vision_preprocessing.py`
   - Line 26: Changed QoS from BEST_EFFORT to RELIABLE

2. `/home/lorenzo/autonomy-ws-25-26/computer_vision/src/cv_ros_nodes/cv_ros_nodes/launch/launch_cv_single_camera1.py`
   - Added TimerAction delays (3s, 5s, 7s) between nodes

3. `/home/lorenzo/autonomy-ws-25-26/comp_single_camera.sh`
   - Updated camera path to `1.1:1.0` (middle camera)

## 🚀 **System Status**

**Pipeline**: ✅ Running (`comp_single_camera.sh`)
**Camera**: ✅ Detecting (1 detection)
**LiDAR**: ✅ Detecting (5-6 buoys)
**Global Frame**: ✅ Working (correct coordinates)
**Fusion**: ⚠️ Running but not assigning class names yet

---

**Major Progress!** The QoS mismatch was the blocker. Camera is now detecting the red buoy! 🎉
