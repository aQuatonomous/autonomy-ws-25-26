#!/bin/bash
# Competition launch (3 cameras): all tasks enabled. Same as comp_single_camera but for all 3 cameras.
# Run from autonomy-ws-25-26. Ctrl+C kills all.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"
PLANNING_WS="${SCRIPT_DIR}/planning"

FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"
# CAMERA_DEVICES: comma-separated. Use /dev/video0,/dev/video1,/dev/video2 or by-path for stable mapping.
#   CAMERA_DEVICES="/dev/video0,/dev/video1,/dev/video2" ./comp.sh
#   CAMERA_DEVICES="/dev/v4l/by-path/...cam0...,/dev/v4l/by-path/...cam1...,/dev/v4l/by-path/...cam2..." ./comp.sh
# Run ./monitor_camera_move.sh to see current by-path devices.
if [ -z "${CAMERA_DEVICES}" ]; then
  CAMERA_DEVICES="/dev/video0,/dev/video1,/dev/video2"
fi
# TASK_ID for planner (1-5)
TASK_ID="${TASK_ID:-3}"

echo "=== Setting camera format (YUYV @ 960x600 @ 15fps) ==="
CAMERA_FPS="${CAMERA_FPS:-15}" CAMERA_DEVICES="${CAMERA_DEVICES}" bash "${CV_WS}/set_camera_fps.sh" || { echo "Warning: set_camera_fps failed (check CAMERA_DEVICES)"; }

echo "=== Sourcing ROS2 and workspaces ==="
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"
source "${PLANNING_WS}/install/setup.bash"

set -m
MAVROS_PID=""
GLOBAL_FRAME_PID=""
LIDAR_PID=""
CV_PID=""
FUSION_PID=""
PLANNER_PID=""
cleanup() {
    echo ""
    echo "=== Stopping MAVROS, global_frame, LiDAR, CV, fusion, and planning ==="
    [ -n "$MAVROS_PID" ] && kill -TERM -"$MAVROS_PID" 2>/dev/null || true
    [ -n "$GLOBAL_FRAME_PID" ] && kill -TERM -"$GLOBAL_FRAME_PID" 2>/dev/null || true
    [ -n "$LIDAR_PID" ] && kill -TERM -"$LIDAR_PID" 2>/dev/null || true
    [ -n "$CV_PID" ]   && kill -TERM -"$CV_PID" 2>/dev/null || true
    [ -n "$FUSION_PID" ] && kill -TERM -"$FUSION_PID" 2>/dev/null || true
    [ -n "$PLANNER_PID" ] && kill -TERM -"$PLANNER_PID" 2>/dev/null || true
    sleep 1
    _KILL="ros2 launch|mavros|global_frame|boat_state_node|detection_to_global|v4l2_camera|v4l2_camera_node|camera0_node|camera1_node|camera2_node|cv_ros_nodes|vision_preprocessing|vision_inference|vision_combiner|maritime_distance|vision_lidar_fusion|task4_supply_processor|indicator_buoy_processor|maritime_distance_estimator|pointcloud_filters|unitree_lidar|lidar_range_filter|buoy_detector|buoy_tracker|global_planner_node"
    pkill -f "$_KILL" 2>/dev/null || true
    sleep 1
    pkill -9 -f "$_KILL" 2>/dev/null || true
    echo "Done."
    exit 130
}
trap cleanup INT TERM

if [[ "$FCU_URL" != /dev/* ]] || [ -e "${FCU_URL%%:*}" ]; then
  echo "=== Launching MAVROS (Pixhawk at ${FCU_URL}) ==="
  ros2 launch mavros apm.launch fcu_url:="${FCU_URL}" &
  MAVROS_PID=$!
  sleep 1
else
  echo "=== Skipping MAVROS (no device at ${FCU_URL%%:*}) ==="
fi

echo "=== Launching global_frame (use_fused_detections:=true) ==="
ros2 launch global_frame global_frame.launch.py use_fused_detections:=true &
GLOBAL_FRAME_PID=$!
sleep 1

echo "=== Launching LiDAR buoy pipeline (no RViz) ==="
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false &
LIDAR_PID=$!

sleep 2
echo "=== Launching CV pipeline (3 cameras, all tasks) ==="
ros2 launch cv_ros_nodes launch_cv.py \
  use_sim:=false \
  resolution:=960,600 \
  conf_threshold:=0.1 \
  preprocess_fps:=5 \
  inference_interval_front:=4 \
  inference_interval_sides:=6 \
  task:=3 \
  enable_task4:=true \
  enable_indicator_buoy:=true \
  enable_number_detection:=true \
  camera_devices:="${CAMERA_DEVICES}" \
  &
CV_PID=$!

sleep 4
echo "=== Starting CV-LiDAR fusion ==="
ros2 run cv_lidar_fusion vision_lidar_fusion &
FUSION_PID=$!

sleep 2
echo "=== Starting global planner (task_id:=${TASK_ID}) ==="
ros2 launch global_planner global_planner.launch.py task_id:="${TASK_ID}" &
PLANNER_PID=$!

echo "=== Pipelines started. MAVROS: $MAVROS_PID  GLOBAL_FRAME: $GLOBAL_FRAME_PID  LiDAR: $LIDAR_PID  CV: $CV_PID  FUSION: $FUSION_PID  PLANNER: $PLANNER_PID ==="
echo "Press Ctrl+C to stop all."
wait -n
cleanup
