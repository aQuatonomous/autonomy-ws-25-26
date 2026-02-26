#!/bin/bash
# Task 1 cheese: no planning. Go straight 5s (steer between gates if seen: red left, green right),
# then turn 180 and return to start. Fused detections only; no local/global planner.
# Same stack as task1 (MAVROS, global_frame, LiDAR, CV, fusion) but runs task1_cheese_node instead.
#
# Env vars: NOLOG=1 disable logging | SOUND=1 also launch sound pipeline

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/loggers comp ran/logging_lib.sh" 2>/dev/null || true

MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"
PLANNING_WS="${SCRIPT_DIR}/planning"

FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"

echo "=== Task 1 cheese: Setting camera format (single camera: YUYV @ 960x600 @ 15fps) ==="
"${SCRIPT_DIR}/set_camera_fps.sh" single || { echo "Warning: set_camera_fps failed"; exit 1; }
CAMERA1_DEVICE="$(cat "${SCRIPT_DIR}/.camera_devices")"

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
CHEESE_PID=""
cleanup() {
    echo ""
    echo "=== Stopping MAVROS, global_frame, LiDAR, CV, fusion, task1_cheese, sound ==="
    stop_sound_pipeline 2>/dev/null || true
    [ -n "$MAVROS_PID" ] && kill -TERM -"$MAVROS_PID" 2>/dev/null || true
    [ -n "$GLOBAL_FRAME_PID" ] && kill -TERM -"$GLOBAL_FRAME_PID" 2>/dev/null || true
    [ -n "$LIDAR_PID" ] && kill -TERM -"$LIDAR_PID" 2>/dev/null || true
    [ -n "$CV_PID" ]   && kill -TERM -"$CV_PID" 2>/dev/null || true
    [ -n "$FUSION_PID" ] && kill -TERM -"$FUSION_PID" 2>/dev/null || true
    [ -n "$CHEESE_PID" ] && kill -TERM -"$CHEESE_PID" 2>/dev/null || true
    sleep 1
    _KILL="ros2 launch|mavros|global_frame|boat_state_node|detection_to_global|v4l2_camera|v4l2_camera_node|camera0_node|camera1_node|camera2_node|cv_ros_nodes|vision_preprocessing|vision_inference|vision_combiner|maritime_distance|vision_lidar_fusion|task4_supply_processor|indicator_buoy_processor|maritime_distance_estimator|pointcloud_filters|unitree_lidar|lidar_range_filter|buoy_detector|buoy_tracker|global_planner_node|task1_cheese_node|audio_capturer|sound_signal|message_node"
    pkill -f "$_KILL" 2>/dev/null || true
    sleep 1
    pkill -9 -f "$_KILL" 2>/dev/null || true
    echo "=== Restarting ROS2 daemon ==="
    ros2 daemon stop 2>/dev/null || true
    sleep 0.5
    ros2 daemon start 2>/dev/null || true
    parse_and_summarize 2>/dev/null || true
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
echo "=== Launching CV pipeline (task:=2, indicator_buoy:=false) ==="
ros2 launch cv_ros_nodes launch_cv_single_camera1.py \
  resolution:=960,600 \
  conf_threshold:=0.3 \
  preprocess_fps:=8 \
  inference_interval_front:=2 \
  task:=2 \
  enable_indicator_buoy:=false \
  enable_task4:=false \
  enable_number_detection:=false \
  camera1_device:="${CAMERA1_DEVICE}" \
  &
CV_PID=$!

sleep 4
echo "=== Starting CV-LiDAR fusion ==="
ros2 run cv_lidar_fusion vision_lidar_fusion &
FUSION_PID=$!

sleep 2
echo "=== Starting Task 1 cheese (no planner: straight 5s, then return to start) ==="
ros2 run global_planner task1_cheese_node --ros-args -p cmd_vel_topic:=/uas1/mavros/setpoint_velocity/cmd_vel_unstamped &
CHEESE_PID=$!

start_sound_pipeline 2>/dev/null || true

echo "=== Task 1 cheese pipelines started. MAVROS: $MAVROS_PID  GLOBAL_FRAME: $GLOBAL_FRAME_PID  LiDAR: $LIDAR_PID  CV: $CV_PID  FUSION: $FUSION_PID  CHEESE: $CHEESE_PID ==="
echo "Press Ctrl+C to stop all."
echo ""
wait
cleanup
