#!/bin/bash
# Competition launch: set camera FPS, then start LiDAR and CV pipelines.
# Run from autonomy-ws-25-26 (or use absolute paths below).
# Ctrl+C kills everything (cameras, inference, lidar, etc.) reliably.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"

# Optional: override camera devices (e.g. after moving cables).
# Run ./monitor_camera_move.sh to see current by-path devices, then:
#   CAMERA_DEVICES="/path1,/path2,/path3" ./comp.sh
# LiDAR is always /dev/ttyUSB0.
CAMERA_DEVICES="${CAMERA_DEVICES:-}"

# Free cameras and serial port from any leftover processes (avoids "Device or resource busy")
echo "=== Stopping any leftover camera/LiDAR/vision processes ==="
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "cv_ros_nodes|v4l2_camera|vision_preprocessing|vision_inference|vision_combiner|maritime_distance" 2>/dev/null || true
pkill -f "pointcloud_filters|unitree_lidar" 2>/dev/null || true
sleep 2

echo "=== Setting camera format (YUYV @ 960x600 @ 15fps) ==="
CAMERA_FPS="${CAMERA_FPS:-15}" bash "${CV_WS}/set_camera_fps.sh" || { echo "Warning: set_camera_fps failed (check camera paths in set_camera_fps.sh or CAMERA_DEVICES)"; }

echo "=== Sourcing ROS2 and workspaces ==="
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"

# So background jobs get their own process groups; Ctrl+C then kills full trees.
set -m
LIDAR_PID=""
CV_PID=""
cleanup() {
    echo ""
    echo "=== Stopping LiDAR and CV pipelines ==="
    [ -n "$LIDAR_PID" ] && kill -TERM -"$LIDAR_PID" 2>/dev/null || true
    [ -n "$CV_PID" ]   && kill -TERM -"$CV_PID" 2>/dev/null || true
    sleep 1
    # Kill by name in case children escaped the process group (ros2 launch spawns nodes separately)
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "cv_ros_nodes|v4l2_camera|vision_preprocessing|vision_inference|vision_combiner|maritime_distance" 2>/dev/null || true
    pkill -f "pointcloud_filters|unitree_lidar|lidar_range_filter|buoy_detector|buoy_tracker" 2>/dev/null || true
    sleep 1
    # Force-kill any stragglers
    pkill -9 -f "ros2 launch" 2>/dev/null || true
    pkill -9 -f "cv_ros_nodes|v4l2_camera|vision_preprocessing|vision_inference|vision_combiner|maritime_distance" 2>/dev/null || true
    pkill -9 -f "pointcloud_filters|unitree_lidar" 2>/dev/null || true
    echo "Done."
    exit 130
}
trap cleanup INT TERM

echo "=== Launching LiDAR buoy pipeline (no RViz, /dev/ttyUSB0) ==="
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false &
LIDAR_PID=$!

sleep 5
echo "=== Launching CV pipeline ==="
ros2 launch cv_ros_nodes launch_cv.py use_sim:=false resolution:=960,600 conf_threshold:=0.1 preprocess_fps:=5 inference_interval_front:=4 inference_interval_sides:=6 &
CV_PID=$!

echo "=== Both pipelines started. LiDAR PID: $LIDAR_PID  CV PID: $CV_PID ==="
echo "Press Ctrl+C to stop both."
wait -n
cleanup