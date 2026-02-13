# Distance Estimation System Changes

## Summary
Replaced the calibration-based distance estimator with a more accurate specs-based approach that uses AR0234 camera specifications directly. This eliminates issues with different capture methods and provides consistent distance estimates.

## Changes Made

### 1. New Maritime Distance Estimator
**File:** `src/cv_ros_nodes/cv_ros_nodes/maritime_distance_estimator.py`
- **Replaces:** `buoy_distance_estimator.py`
- **Subscribes to:** `/combined/detection_info` (instead of per-camera topics)
- **Publishes to:** `/combined/detection_info_with_distance`

**Key improvements:**
- Uses AR0234 camera specifications directly (3.56mm focal length, 3μm pixels, 1920×1200)
- No dependency on calibration files that can vary by capture method
- Works consistently whether images come from video extraction, photo apps, or terminal capture
- Focal length: `fy = 3.56mm / 0.003mm = 1186.67 pixels` (vs ~512 from calibration)

**Distance formula:** `distance_m = (fy * reference_height_m) / height_px`

### 2. Enhanced Buoy Classification
**File:** `src/cv_ros_nodes/cv_ros_nodes/vision_combiner.py`
- **Updated:** `_resolve_class_name()` method with aspect ratio logic

**Aspect ratio classification:**
- **Square-ish** (`width/height ≥ 0.7`): `red_buoy`, `green_buoy` (1 ft reference)  
- **Tall/vertical** (`width/height < 0.7`): `red_pole_buoy`, `green_pole_buoy` (39 in reference)

This prevents distance estimation errors when the detector mislabels pole buoys as regular buoys or vice versa.

### 3. Updated Package Configuration
**File:** `src/cv_ros_nodes/setup.py`
- **Changed:** `buoy_distance_estimator` → `maritime_distance_estimator`

## Reference Dimensions Used
Based on RoboBoat 2026 specifications:

| Object Class | Reference Height | Source |
|--------------|------------------|---------|
| `red_buoy`, `green_buoy`, `black_buoy`, `yellow_buoy` | 0.305m (1 ft) | Polyform A-0/A-2 above waterline |
| `red_pole_buoy`, `green_pole_buoy` | 0.991m (39 in) | Taylor Made Sur-Mark above waterline |
| `dock` | 0.406m (16 in) | Dock height specification |
| `digit_1`, `digit_2`, `digit_3` | 0.610m (24 in) | Banner dimensions |
| `red_indicator_buoy`, `green_indicator_buoy` | 0.432m (17 in) | Total buoy height |
| `cross`, `triangle` | 0.203m (8 in) | Diamond vertex-to-vertex |
| `yellow_supply_drop`, `black_supply_drop` | 0.406m (16 in) | Width (height not specified) |

## Output Format
The new node adds these fields to each detection:
```json
{
    "class_name": "red_buoy",
    "bbox": [x1, y1, x2, y2],
    "distance_m": 3.32,
    "distance_ft": 10.9,
    "reference_height_m": 0.305,
    "camera_id": 0,
    "bearing_deg": -15.2,
    "confidence": 0.85
}
```

## Performance Comparison
**Test case:** 1 ft buoy at 9 ft 5 in (2.87 m actual distance)

| Method | Estimated Distance | Accuracy |
|--------|-------------------|----------|
| **Old calibration-based** | 4.66 m (15.3 ft) | ❌ 62% error |
| **New specs-based** | 3.32 m (10.9 ft) | ✅ 16% error |
| **Ground truth** | 2.87 m (9.4 ft) | — |

## Usage

### Running the new system:
```bash
# Start the maritime distance estimator (replaces buoy_distance_estimator)
ros2 run cv_ros_nodes maritime_distance_estimator

# The vision_combiner now includes aspect ratio logic automatically
ros2 run cv_ros_nodes vision_combiner
```

### Topics:
- **Input:** `/combined/detection_info` (from vision_combiner)
- **Output:** `/combined/detection_info_with_distance` (with distance estimates)

### Standalone testing:
```bash
# Test with camera specs only (no ROS)
cd cv_scripts
python3 distance_from_specs_only.py image.jpg --override-class red_buoy
```

## Migration Notes
- **Old node:** Subscribe to per-camera topics → estimate distance → publish per-camera
- **New node:** Subscribe to combined topic → estimate distance → publish combined
- **Downstream consumers:** Change from `/camera0/detection_info_with_distance` to `/combined/detection_info_with_distance`
- **No calibration files needed:** System now works out-of-the-box with camera specifications

## Documentation Updated
- `maritime_distance_estimator.py`: Full class and method documentation
- `vision_combiner.py`: Updated docstring and method documentation  
- `setup.py`: Updated entry points
- `run_inference_and_distance.py`: Updated comments