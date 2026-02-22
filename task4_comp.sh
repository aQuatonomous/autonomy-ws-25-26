#!/bin/bash
# Task 4 - Supply Drop (vessels). Water delivery to yellow stationary vessels.
# CV: enable_task4:=true (vessel detection). Planner: task_id:=4.
# Run from autonomy-ws-25-26. Ctrl+C kills all.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"
PLANNING_WS="${SCRIPT_DIR}/planning"

FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"

echo "=== Task 4: Setting camera format (single camera) ==="
"${SCRIPT_DIR}/set_camera_fps.sh" single || { echo "Warning: set_camera_fps failed"; exit 1; }
CAMERA1_DEVICE="$(cat "${SCRIPT_DIR}/.camera_devices")"

echo "=== Sourcing ROS2 and workspaces ==="
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"
source "${PLANNING_WS}/install/setup.bash"

set -m
MAVROS_PID=""; GLOBAL_FRAME_PID=""; LIDAR_PID=""; CV_PID=""; FUSION_PID=""; PLANNER_PID=""
cleanup() {
    echo ""; echo "=== Stopping all ==="
    [ -n "$MAVROS_PID" ] && kill -TERM -"$MAVROS_PID" 2>/dev/null || true
    [ -n "$GLOBAL_FRAME_PID" ] && kill -TERM -"$GLOBAL_FRAME_PID" 2>/dev/null || true
    [ -n "$LIDAR_PID" ] && kill -TERM -"$LIDAR_PID" 2>/dev/null || true
    [ -n "$CV_PID" ] && kill -TERM -"$CV_PID" 2>/dev/null || true
    [ -n "$FUSION_PID" ] && kill -TERM -"$FUSION_PID" 2>/dev/null || true
    [ -n "$PLANNER_PID" ] && kill -TERM -"$PLANNER_PID" 2>/dev/null || true
    sleep 1
    _KILL="ros2 launch|mavros|global_frame|boat_state_node|detection_to_global|v4l2_camera|camera0_node|camera1_node|camera2_node|cv_ros_nodes|vision_preprocessing|vision_inference|vision_combiner|maritime_distance|vision_lidar_fusion|task4_supply_processor|indicator_buoy_processor|maritime_distance_estimator|pointcloud_filters|unitree_lidar|lidar_range_filter|buoy_detector|buoy_tracker|global_planner_node"
    pkill -f "$_KILL" 2>/dev/null || true; sleep 1; pkill -9 -f "$_KILL" 2>/dev/null || true
    echo "Done."; exit 130
}
trap cleanup INT TERM

if [[ "$FCU_URL" != /dev/* ]] || [ -e "${FCU_URL%%:*}" ]; then
  echo "=== Launching MAVROS ==="
  ros2 launch mavros apm.launch fcu_url:="${FCU_URL}" &
  MAVROS_PID=$!; sleep 1
else
  echo "=== Skipping MAVROS ==="
fi

echo "=== Launching global_frame ==="
ros2 launch global_frame global_frame.launch.py use_fused_detections:=true &
GLOBAL_FRAME_PID=$!; sleep 1

echo "=== Launching LiDAR buoy pipeline ==="
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false &
LIDAR_PID=$!

sleep 2
echo "=== Launching CV (Task 4: enable_task4:=true, vessels) ==="
ros2 launch cv_ros_nodes launch_cv_single_camera1.py \
  resolution:=960,600 \
  conf_threshold:=0.3 \
  preprocess_fps:=8 \
  inference_interval_front:=2 \
  task:=4 \
  enable_task4:=true \
  enable_indicator_buoy:=true \
  enable_number_detection:=false \
  camera1_device:="${CAMERA1_DEVICE}" \
  &
CV_PID=$!

sleep 4
echo "=== Starting CV-LiDAR fusion ==="
ros2 run cv_lidar_fusion vision_lidar_fusion &
FUSION_PID=$!

sleep 2
echo "=== Starting Task 4 planner (task_id:=4) ==="
ros2 launch global_planner global_planner.launch.py task_id:=4 &
PLANNER_PID=$!

echo "=== Task 4 started. Ctrl+C to stop. ==="
echo "(If any launch exits early, the rest keep running until Ctrl+C.)"
echo ""
wait
cleanup
