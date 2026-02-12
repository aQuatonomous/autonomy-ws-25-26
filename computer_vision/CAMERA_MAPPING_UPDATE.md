# Camera Device Mapping Update

This document summarizes the changes made to use persistent USB device paths for camera mapping.

## What Changed

### 1. Launch File Updated
**File:** `src/cv_ros_nodes/cv_ros_nodes/launch/launch_cv.py`

**Before:**
```python
default_value='/dev/video0,/dev/video2,/dev/video4'
```

**After:**
```python
default_value='/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0'
```

### 2. Documentation Updated
- **NODES_AND_CAMERAS.md**: Updated all camera device references
- **README.md**: Updated FPS setting commands
- **DESIGN_STRATEGY.md**: No changes needed (examples only)

### 3. New Convenience Script
**File:** `set_camera_fps.sh`
- Automatically sets FPS on all cameras using persistent paths
- Provides verification and error checking
- Usage: `./set_camera_fps.sh`

## Camera Mapping

| Camera | USB Port | Persistent Device Path | ROS Namespace |
|--------|----------|----------------------|---------------|
| Camera 0 | 1.2.2 | `/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0` | `/camera0` |
| Camera 1 | 1.2.3 | `/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0` | `/camera1` |
| Camera 2 | 1.2.4 | `/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0` | `/camera2` |

## Benefits

1. **Persistent Mapping**: Cameras will always get the same ROS namespace regardless of:
   - Boot order
   - USB plug/unplug order  
   - System reboots
   - Other USB devices being connected

2. **Physical Port Based**: Each camera is tied to a specific USB port:
   - Camera in port 1.2.2 → always becomes `/camera0`
   - Camera in port 1.2.3 → always becomes `/camera1`
   - Camera in port 1.2.4 → always becomes `/camera2`

3. **No Additional Setup**: Uses existing kernel-provided symlinks

## Usage

### Quick Start (Same as Before)
```bash
# Set FPS (using new script)
./set_camera_fps.sh

# Launch pipeline (no changes needed)
ros2 launch cv_ros_nodes launch_cv.py
```

### Manual FPS Setting
```bash
# Using persistent paths (recommended)
for d in /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0 /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0 /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0; do v4l2-ctl -d "$d" --set-parm 15; done

# Or using current video numbers (may change)
for d in /dev/video0 /dev/video2 /dev/video4; do v4l2-ctl -d "$d" --set-parm 15; done
```

### Override Camera Devices (if needed)
```bash
ros2 launch cv_ros_nodes launch_cv.py camera_devices:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0
```

## Troubleshooting

### Check Current Device Mapping
```bash
ls -la /dev/v4l/by-path/
```

### Verify Camera Detection
```bash
v4l2-ctl --list-devices
```

### Check Which USB Port a Camera Is On
```bash
udevadm info --query=all --name=/dev/video0 | grep ID_PATH
```

## Backward Compatibility

The old `/dev/video*` paths still work if you override the `camera_devices` parameter:
```bash
ros2 launch cv_ros_nodes launch_cv.py camera_devices:=/dev/video0,/dev/video2,/dev/video4
```

However, using persistent paths is strongly recommended for production use.