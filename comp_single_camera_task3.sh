#!/bin/bash
# Task 3 - Emergency Response Sprint (Speed Challenge) - Single camera competition launch.
#
# Task 3.2.3: ASV locates gate (red + green buoys), passes through, maneuvers around
# the yellow buoy as indicated by the color indicator buoy, then exits through the gate.
#   - Red color indicator  = circle yellow buoy from the right (counter-clockwise)
#   - Green color indicator = circle yellow buoy from the left (clockwise)
#
# CV pipeline parameters set for Task 3:
#   - task:=3               → Buoy reference dimensions: green/red/yellow 1 ft, black 0.5 ft (maritime_distance_estimator)
#   - enable_indicator_buoy:=true → Color indicator buoy processor (red/green diamond on white buoy)
#   - conf_threshold:=0.1   → YOLO detection threshold for gate buoys (red_buoy, green_buoy) and yellow_buoy
#   - Single camera (camera1) only; LiDAR buoy pipeline + CV-LiDAR fusion for fused_buoys.
#
# Run from autonomy-ws-25-26. Ctrl+C kills all (MAVROS, global_frame, LiDAR, CV, fusion, planning).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"
PLANNING_WS="${SCRIPT_DIR}/planning"

FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"

echo "=== Task 3: Setting camera format (single camera: YUYV @ 960x600 @ 15fps) ==="
"${SCRIPT_DIR}/set_camera_fps.sh" single || { echo "Warning: set_camera_fps failed (edit set_camera_fps.sh or run ./monitor_camera_move.sh)"; exit 1; }
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

echo "=== Launching global_frame (boat_state_node + detection_to_global_node, use_fused_detections:=true) ==="
ros2 launch global_frame global_frame.launch.py use_fused_detections:=true &
GLOBAL_FRAME_PID=$!
sleep 1

echo "=== Launching LiDAR buoy pipeline (no RViz, /dev/ttyUSB0) ==="
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false &
LIDAR_PID=$!

sleep 2
echo "=== Launching CV pipeline (Task 3: camera1, task:=3, indicator buoy enabled) ==="
ros2 launch cv_ros_nodes launch_cv_single_camera1.py \
  resolution:=960,600 \
  conf_threshold:=0.1 \
  preprocess_fps:=5 \
  inference_interval_front:=4 \
  task:=3 \
  enable_indicator_buoy:=true \
  camera1_device:="${CAMERA1_DEVICE}" \
  &
CV_PID=$!

sleep 4
echo "=== Starting CV-LiDAR fusion ==="
ros2 run cv_lidar_fusion vision_lidar_fusion &
FUSION_PID=$!

sleep 2
echo "=== Starting Task 3 global planner (task_id:=3) ==="
ros2 launch global_planner global_planner.launch.py task_id:=3 &
PLANNER_PID=$!

echo "=== Task 3 pipelines started. MAVROS: $MAVROS_PID  GLOBAL_FRAME: $GLOBAL_FRAME_PID  LiDAR: $LIDAR_PID  CV: $CV_PID  FUSION: $FUSION_PID  PLANNER: $PLANNER_PID ==="
echo "Press Ctrl+C to stop all. All node logs appear below (interleaved)."
echo ""
wait -n
cleanup
