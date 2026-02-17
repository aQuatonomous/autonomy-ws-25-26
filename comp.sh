#!/bin/bash
# Competition launch: set camera FPS, then start LiDAR and CV pipelines.
# Run from autonomy-ws-25-26 (or use absolute paths below).
# Ctrl+C kills everything (cameras, inference, lidar, etc.) reliably.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"

# Optional: override camera or LiDAR ports (e.g. after moving cables).
# Run ./monitor_camera_move.sh to see current camera by-path devices, then:
#   CAMERA_DEVICES="/path1,/path2,/path3" LIDAR_PORT="/dev/serial/by-path/..." ./comp.sh
CAMERA_DEVICES="${CAMERA_DEVICES:-}"
LIDAR_PORT="${LIDAR_PORT:-}"

echo "=== Camera positions ==="
bash "${SCRIPT_DIR}/monitor_camera_move.sh" 2>/dev/null || true

echo "=== Setting camera format (YUYV @ 960x600 @ 15fps) ==="
CAMERA_FPS="${CAMERA_FPS:-15}" bash "${CV_WS}/set_camera_fps.sh" || { echo "Warning: set_camera_fps failed (check camera paths in set_camera_fps.sh or CAMERA_DEVICES)"; }

echo "=== Sourcing ROS2 and workspaces ==="
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"

echo "=== Launching LiDAR buoy pipeline (no RViz) ==="
LIDAR_ARGS="launch_rviz:=false"
[ -n "$LIDAR_PORT" ] && LIDAR_ARGS="$LIDAR_ARGS lidar_port:=$LIDAR_PORT"
ros2 launch pointcloud_filters buoy_pipeline.launch.py $LIDAR_ARGS &
LIDAR_PID=$!

sleep 5
echo "=== Launching CV pipeline ==="
ros2 launch cv_ros_nodes launch_cv.py use_sim:=false resolution:=960,600 conf_threshold:=0.1 preprocess_fps:=5 inference_interval_front:=4 inference_interval_sides:=6 &
CV_PID=$!

echo "=== Both pipelines started. LiDAR PID: $LIDAR_PID  CV PID: $CV_PID ==="
echo "Press Ctrl+C to stop both."
wait -n
kill $LIDAR_PID $CV_PID 2>/dev/null