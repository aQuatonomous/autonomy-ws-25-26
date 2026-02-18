#!/bin/bash

# Set camera resolution and FPS using persistent USB device paths.
# Accepts CAMERA_DEVICES as comma-separated string (e.g., "/path1,/path2,/path3").
# Camera mapping is handled by the calling script (e.g., comp.sh).
# FPS: 15 fps at 960x600. Launch uses sequential startup (0s, 2s, 4s) to avoid USB race.

RESOLUTION="${CAMERA_RESOLUTION:-960,600}"
FPS="${CAMERA_FPS:-15}"
IFS=',' read -r WIDTH HEIGHT <<< "$RESOLUTION"
echo "Setting cameras to ${WIDTH}x${HEIGHT} YUYV @ ${FPS} FPS..."

# Parse CAMERA_DEVICES from comma-separated string to array
# If CAMERA_DEVICES is not set, script will exit with error
if [ -z "${CAMERA_DEVICES}" ]; then
    echo "Error: CAMERA_DEVICES environment variable is not set."
    echo "Set it in your calling script (e.g., comp.sh) as a comma-separated string:"
    echo "  CAMERA_DEVICES=\"/path1,/path2,/path3\""
    exit 1
fi

# Convert comma-separated string to array
IFS=',' read -ra CAMERA_DEVICES_ARRAY <<< "${CAMERA_DEVICES}"
CAMERA_DEVICES=("${CAMERA_DEVICES_ARRAY[@]}")

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
echo "Done! Configured ${#CAMERA_DEVICES[@]} camera(s)."
