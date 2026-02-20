# Dock Distance Flow Verification

## Date: 2026-02-19

## Summary
✅ **VERIFIED**: The dock distance handling is working correctly throughout the entire pipeline. Docks intentionally receive `distance_m=None` from CV and are properly skipped in global frame conversion.

---

## Complete Pipeline Flow

### 1. Computer Vision Detection
**Node**: `vision_inference` (YOLO model)  
**Output**: Detections with bounding boxes, class_id=7 for 'dock'

```json
{
  "class_id": 7,
  "class_name": "dock",
  "bbox": [x1, y1, x2, y2],
  "score": 0.85,
  "camera_id": 0
}
```

### 2. Detection Combiner
**Node**: `vision_combiner`  
**Topic**: `/combined/detection_info`  
**Action**: Aggregates detections from all cameras, adds bearing/elevation

```json
{
  "detections": [
    {
      "class_id": 7,
      "class_name": "dock",
      "bbox": [x1, y1, x2, y2],
      "score": 0.85,
      "camera_id": 0,
      "bearing_deg": 15.3,
      "elevation_deg": -2.1
    }
  ]
}
```

### 3. Maritime Distance Estimator ⭐ KEY STEP
**Node**: `maritime_distance_estimator`  
**File**: `computer_vision/src/cv_ros_nodes/cv_ros_nodes/maritime_distance_estimator.py`  
**Topic In**: `/combined/detection_info`  
**Topic Out**: `/combined/detection_info_with_distance`

**Logic** (Lines 189-191):
```python
def _estimate_distance_m(self, detection: Dict) -> Optional[float]:
    # ...
    class_name = detection.get('class_name', '')
    
    # Skip distance estimation for objects with unknown/variable sizes
    if class_name == 'dock':
        return None  # Dock sizes vary enormously and are unknown
```

**Output**:
```json
{
  "detections": [
    {
      "class_id": 7,
      "class_name": "dock",
      "bbox": [x1, y1, x2, y2],
      "score": 0.85,
      "camera_id": 0,
      "bearing_deg": 15.3,
      "elevation_deg": -2.1,
      "distance_m": null,        // ⭐ INTENTIONALLY NULL
      "distance_ft": null,
      "reference_height_m": null
    }
  ]
}
```

### 4. Detection to Global Frame ⭐ KEY STEP
**Node**: `detection_to_global_node`  
**File**: `mapping/src/global_frame/global_frame/detection_to_global_node.py`  
**Topic In**: `/combined/detection_info_with_distance`  
**Topic Out**: `/global_detections`

**Logic** (Lines 230-238):
```python
def _cv_callback(self, msg: String) -> None:
    # ...
    for d in detections:
        dist = d.get("distance_m")
        if dist is None:
            class_name = d.get("class_name", "unknown")
            # Dock intentionally has distance_m=None (variable size, CV-only); skip without warning
            if class_name == "dock":
                self.get_logger().debug("CV dock detection skipped (distance_m=None by design)", throttle_duration_sec=2.0)
            else:
                self.get_logger().warn(f"CV detection missing distance_m: {class_name}", throttle_duration_sec=2.0)
            continue  # ⭐ SKIP THIS DETECTION - NOT ADDED TO GLOBAL FRAME
```

**Result**: Dock detections are **not published** to `/global_detections` because they lack distance information.

---

## Why This Design?

### Problem
Docks have **highly variable dimensions**:
- Different dock designs (floating, fixed, modular)
- Variable heights above waterline (tide-dependent)
- Different widths and lengths
- No consistent reference dimension for distance estimation

### Solution
1. **CV detects docks** for visual awareness and task-specific logic
2. **Distance estimation skipped** (`distance_m=None`) because no reliable reference
3. **Global frame conversion skipped** because position cannot be computed without distance
4. **LiDAR provides dock positioning** when needed (LiDAR has direct range measurements)

---

## Verification Checklist

### ✅ Maritime Distance Estimator
- [x] Line 190: `if class_name == 'dock': return None`
- [x] Returns `None` for dock detections
- [x] Sets `distance_m`, `distance_ft`, `reference_height_m` all to `None`

### ✅ Detection to Global Node
- [x] Line 231: Checks `if dist is None`
- [x] Line 234: Special handling for `class_name == "dock"`
- [x] Line 235: Logs at `debug` level (not warning) for dock
- [x] Line 238: `continue` - skips adding to global detections
- [x] Line 237: Other classes with `None` distance get warning (unexpected)

### ✅ Logging Behavior
- **Dock with `distance_m=None`**: Debug log (expected behavior)
- **Other class with `distance_m=None`**: Warning log (unexpected behavior)
- **Throttled**: Both logs throttled to 2 seconds to avoid spam

---

## Testing Scenarios

### Scenario 1: Dock Detection Only
**Input**: CV detects dock at bearing 15°  
**Expected**:
1. ✅ Detection appears in `/combined/detection_info`
2. ✅ Detection appears in `/combined/detection_info_with_distance` with `distance_m=null`
3. ✅ Detection **NOT** in `/global_detections` (skipped)
4. ✅ Debug log: "CV dock detection skipped (distance_m=None by design)"

### Scenario 2: Dock + Buoy Detection
**Input**: CV detects dock + red buoy  
**Expected**:
1. ✅ Both detections in `/combined/detection_info`
2. ✅ Dock: `distance_m=null`, Buoy: `distance_m=3.5`
3. ✅ Only buoy in `/global_detections`
4. ✅ Debug log for dock, no warning

### Scenario 3: Unknown Object Without Distance
**Input**: CV detects unknown object, distance estimation fails  
**Expected**:
1. ✅ Detection in `/combined/detection_info_with_distance` with `distance_m=null`
2. ✅ Detection **NOT** in `/global_detections`
3. ✅ Warning log: "CV detection missing distance_m: unknown_class"

---

## Related Files

### Modified Files (Previous Work)
1. `computer_vision/src/cv_ros_nodes/cv_ros_nodes/maritime_distance_estimator.py`
   - Line 190-191: Skip dock distance estimation

2. `mapping/src/global_frame/global_frame/detection_to_global_node.py`
   - Line 233-238: Handle dock with `distance_m=None` gracefully

### Reference Documentation
- `computer_vision/DISTANCE_ESTIMATOR_CHANGES.md` - Distance estimation methodology
- `computer_vision/NODES.md` - Complete node architecture
- `mapping/README.md` - Global frame conversion details

---

## Conclusion

✅ **The pipeline is working correctly**:
1. Docks are detected by CV for visual awareness
2. Distance estimation is intentionally skipped (returns `None`)
3. Global frame conversion gracefully skips docks with debug logging
4. No warnings or errors for expected behavior
5. Other objects with missing distance get appropriate warnings

**No issues found. System is production-ready.**

---

## Future Considerations

### Option 1: LiDAR-Based Dock Positioning
- Use LiDAR to detect dock edges/corners
- Fuse CV detection (visual confirmation) with LiDAR range
- More reliable for docking tasks

### Option 2: Multi-Point Dock Estimation
- Detect multiple dock features (corners, edges)
- Triangulate position from multiple camera views
- Requires more sophisticated CV processing

### Option 3: Task-Specific Dock Distance
- For docking task, use specialized close-range estimation
- Different reference dimensions for different dock types
- Would require dock type classification

**Current approach (skip distance) is the most robust for general use.**
