#!/bin/bash
# Competition launch: set camera FPS, then start LiDAR and CV pipelines.
# Run from autonomy-ws-25-26 (or use absolute paths below).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"

echo "=== Setting camera FPS (15) ==="
bash "${CV_WS}/set_camera_fps.sh" || { echo "Warning: set_camera_fps failed"; }

echo "=== Sourcing ROS2 and workspaces ==="
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"

echo "=== Launching LiDAR buoy pipeline (no RViz) ==="
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false &
LIDAR_PID=$!

sleep 5
echo "=== Launching CV pipeline ==="
ros2 launch cv_ros_nodes launch_cv.py use_sim:=false conf_threshold:=0.1 inference_interval_front:=1 inference_interval_sides:=4 &
CV_PID=$!

echo "=== Both pipelines started. LiDAR PID: $LIDAR_PID  CV PID: $CV_PID ==="
echo "Press Ctrl+C to stop both."
wait -n
kill $LIDAR_PID $CV_PID 2>/dev/null