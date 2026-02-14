#!/bin/bash

# Set camera FPS using persistent USB device paths.
# Sets all 3 cameras to 15 FPS. Paths below are for the Think (development machine);
# on other hardware (e.g. Jetson) run: v4l2-ctl --list-devices
# and edit CAMERA_DEVICES to match your /dev/v4l/by-path/... paths.

echo "Setting camera FPS to 15 using persistent USB paths..."

# Define the persistent device paths
CAMERA_DEVICES=(
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0"  # Camera 0 (USB port 1.2.2)
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0"  # Camera 1 (USB port 1.2.3)
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0"  # Camera 2 (USB port 1.2.4)
)

# Set FPS for each camera
for i in "${!CAMERA_DEVICES[@]}"; do
    device="${CAMERA_DEVICES[$i]}"
    if [ -e "$device" ]; then
        echo "Setting FPS for Camera $i: $device"
        v4l2-ctl -d "$device" --set-parm 15
        if [ $? -eq 0 ]; then
            echo "✓ Camera $i FPS set successfully"
        else
            echo "✗ Failed to set FPS for Camera $i"
        fi
    else
        echo "⚠ Camera $i device not found: $device"
        echo "  Check if the camera is plugged into the correct USB port"
    fi
    echo
done

echo "Verifying FPS settings..."
for i in "${!CAMERA_DEVICES[@]}"; do
    device="${CAMERA_DEVICES[$i]}"
    if [ -e "$device" ]; then
        echo "=== Camera $i FPS: $device ==="
        v4l2-ctl -d "$device" --get-parm
    fi
done

echo "Done! You can now launch the ROS pipeline with:"
echo "ros2 launch cv_ros_nodes launch_cv.py"