#!/bin/bash

# Set camera resolution and FPS using persistent USB device paths.
# Camera positions (by-path): Cam0 = 1.2, Cam1 = 1.1, Cam2 = 1.4.2
# If you move cables, run ../monitor_camera_move.sh to see new paths and update CAMERA_DEVICES below.
# FPS: 15 fps at 960x600. Launch uses sequential startup (0s, 2s, 4s) to avoid USB race.

RESOLUTION="${CAMERA_RESOLUTION:-960,600}"
FPS="${CAMERA_FPS:-15}"
IFS=',' read -r WIDTH HEIGHT <<< "$RESOLUTION"
echo "Setting cameras to ${WIDTH}x${HEIGHT} YUYV @ ${FPS} FPS..."

# By-path: 1.2→Cam0, 1.1→Cam1, 1.4.2→Cam2 (see ../monitor_camera_move.sh if paths change)
CAMERA_DEVICES=(
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.2:1.0-video-index0"   # Camera 0
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2:1.0-video-index0"   # Camera 1
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.3:1.0-video-index0" # Camera 2
)

# Set resolution and FPS for each camera
for i in "${!CAMERA_DEVICES[@]}"; do
    device="${CAMERA_DEVICES[$i]}"
    if [ -e "$device" ]; then
        echo "Camera $i: $device"
        v4l2-ctl -d "$device" --set-fmt-video=width="$WIDTH",height="$HEIGHT",pixelformat=YUYV
        v4l2-ctl -d "$device" --set-parm "$FPS"
        if [ $? -eq 0 ]; then
            echo "  ✓ ${WIDTH}x${HEIGHT} @ ${FPS} fps"
        else
            echo "  ✗ Failed"
        fi
    else
        echo "⚠ Camera $i not found: $device (run ../monitor_camera_move.sh to see current by-path devices)"
    fi
    echo
done

echo "Verifying..."
for i in "${!CAMERA_DEVICES[@]}"; do
    device="${CAMERA_DEVICES[$i]}"
    if [ -e "$device" ]; then
        echo "=== Camera $i ==="
        v4l2-ctl -d "$device" --get-fmt-video 2>/dev/null | grep -E "Width|Height|Pixel"
        v4l2-ctl -d "$device" --get-parm 2>/dev/null
    fi
done

echo ""
echo "Done! Ports: 1.4.2→Cam0, 1.2→Cam1, 1.4.3→Cam2. Launch with:"
echo "  ros2 launch cv_ros_nodes launch_cv.py camera_devices:=/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.3:1.0-video-index0"