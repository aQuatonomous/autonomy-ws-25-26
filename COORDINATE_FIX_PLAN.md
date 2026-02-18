# COORDINATE SYSTEM FIX PLAN

## IDENTIFIED PROBLEMS

### 1. **Compass Heading Conversion is WRONG**
- **Current**: `boat_state_node.py` line 149: `self._last_heading_rad = math.radians(hdg_deg - 90.0)`
- **Issue**: Converts compass (0°=North, 90°=East) to "angle from East" which gives WRONG results
- **Example**: Boat facing North (compass=0°) → converts to -90° → -1.57 rad → boat thinks it's facing WEST!
- **Actual output we saw**: `heading_rad: 4.126` (≈236°) when compass was 326°
- **Calculation**: 326° - 90° = 236° = 4.12 rad ✓ (confirms the bug)

### 2. **Bearing Angle Wrapping is BROKEN**
- **Current**: `detection_to_global_node.py` line 194: `alpha = (self._boat_heading_rad or 0.0) + bearing_rad`
- **Issue**: No angle normalization → produces angles > 2π (>360°) like 415° we saw
- **Result**: Invalid bearing_global_rad values in output

### 3. **LiDAR TF Transform Quaternion is WRONG**
- **Current**: `buoy_pipeline.launch.py` line 56: `'--qx', '0.2152', '--qy', '-0.1438', '--qz', '0.8031', '--qw', '0.5369'`
- **Issue**: This quaternion represents 112.5° CW yaw + 30° CCW pitch
- **Problem**: LiDAR is pointed FORWARD (0° yaw) and DOWN (30° pitch), NOT rotated 112.5° sideways!
- **Correct**: Should be 0° yaw, 30° pitch down = qx=0.2588, qy=0, qz=0, qw=0.9659

### 4. **Coordinate Convention Confusion**
- **ROS standard**: +X forward, +Y left, +Z up
- **Compass**: 0°=North, 90°=East, 180°=South, 270°=West
- **Current code**: Mixes "angle from East" with "boat forward" → chaos

## CORRECT COORDINATE CONVENTIONS

### **Global Frame (map)**
- **Origin**: (0, 0) = starting GPS position (or 0,0 indoors)
- **Axes**: +East = +X, +North = +Y
- **Heading**: 0° = East, 90° = North, 180° = West, 270° = South (standard math/ENU convention)

### **Boat Frame (base_link)**
- **ROS standard**: +X = forward, +Y = left, +Z = up
- **Heading**: 0° = boat forward (+X direction)
- **Bearing**: Angle from boat forward (0° = straight ahead)

### **Compass to Boat Heading Conversion**
- **Compass**: 0° = North, 90° = East, 180° = South, 270° = West
- **Boat heading in ENU**: `heading_rad = math.radians(90.0 - compass_deg)`
  - North (0°) → 90° ENU (pointing +Y)
  - East (90°) → 0° ENU (pointing +X)
  - South (180°) → -90° or 270° ENU (pointing -Y)
  - West (270°) → 180° ENU (pointing -X)

## FIXES REQUIRED

### Fix 1: Correct compass heading conversion in `boat_state_node.py`
```python
# OLD (WRONG):
self._last_heading_rad = math.radians(hdg_deg - 90.0)

# NEW (CORRECT):
self._last_heading_rad = math.radians(90.0 - hdg_deg)
```

### Fix 2: Add angle normalization in `detection_to_global_node.py`
```python
def _normalize_angle(angle_rad: float) -> float:
    """Normalize angle to [-π, π]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad

# In _to_global and _lidar_callback:
alpha = _normalize_angle(self._boat_heading_rad + bearing_rad)
bearing_global_rad = _normalize_angle(alpha)
```

### Fix 3: Correct LiDAR TF quaternion in `buoy_pipeline.launch.py`
```python
# OLD (WRONG - 112.5° CW yaw + 30° pitch):
'--qx', '0.2152', '--qy', '-0.1438', '--qz', '0.8031', '--qw', '0.5369'

# NEW (CORRECT - 0° yaw, 30° pitch down):
'--qx', '0.2588', '--qy', '0', '--qz', '0', '--qw', '0.9659'
```

### Fix 4: Verify bearing calculation in LiDAR detections
- LiDAR outputs bearing in base_link frame (0° = forward)
- This should be correct already, but verify the sign convention

## TESTING PLAN

### Test 1: Compass Heading
1. Point boat NORTH → compass should read ~0°
2. Check `/boat_pose` → `heading_rad` should be ~1.57 (90° = North in ENU)
3. Point boat EAST → compass should read ~90°
4. Check `/boat_pose` → `heading_rad` should be ~0.0 (0° = East in ENU)

### Test 2: LiDAR Detection Alignment
1. Place single buoy directly in front of boat (North)
2. Point boat North
3. Check `/tracked_buoys_json` → bearing should be ~0° (forward)
4. Check `/global_detections` → should show buoy to the NORTH (positive Y)

### Test 3: Angle Wrapping
1. Run full pipeline
2. Check `/global_detections` → all `bearing_global_rad` should be in [-π, π] or [0, 2π]
3. No angles > 6.28 (360°)

### Test 4: Camera-LiDAR Alignment
1. Place red buoy in front
2. Both camera and LiDAR should detect it at same bearing
3. Fusion should merge them correctly

## IMPLEMENTATION ORDER

1. ✅ Create this plan
2. Fix compass heading conversion in `boat_state_node.py`
3. Fix LiDAR TF quaternion in `buoy_pipeline.launch.py`
4. Add angle normalization to `detection_to_global_node.py`
5. Rebuild all packages
6. Test compass heading (Test 1)
7. Test LiDAR alignment (Test 2)
8. Test angle wrapping (Test 3)
9. Test full pipeline with camera (Test 4)
10. Verify single red buoy detection in front matches reality
