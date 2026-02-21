#!/bin/bash
# Competition launch: set camera FPS, then start LiDAR and CV pipelines.
# Run from autonomy-ws-25-26 (or use absolute paths below).
# Ctrl+C kills everything (cameras, inference, lidar, etc.) reliably.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"

# Camera device mapping. Override if needed (e.g. after moving cables):
#   CAMERA_DEVICES="/path1,/path2,/path3" ./comp.sh
# Run ./monitor_camera_move.sh to see current by-path devices.
# Default matches launch_cv.py default (Cam0, Cam1, Cam2).
# LiDAR is always /dev/ttyUSB0. Pixhawk is /dev/ttyACM0 (override with FCU_URL, e.g. /dev/ttyACM0:57600).
FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}"
if [ -z "${CAMERA_DEVICES}" ]; then
  CAMERA_DEVICES="/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0,"\
"/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4:1.0-video-index0,"\
"/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0"
fi


echo "=== Setting camera format (YUYV @ 960x600 @ 15fps) ==="
CAMERA_FPS="${CAMERA_FPS:-15}" CAMERA_DEVICES="${CAMERA_DEVICES}" bash "${CV_WS}/set_camera_fps.sh" || { echo "Warning: set_camera_fps failed (check camera paths or CAMERA_DEVICES)"; }

echo "=== Sourcing ROS2 and workspaces ==="
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"

# So background jobs get their own process groups; Ctrl+C then kills full trees.
set -m
MAVROS_PID=""
GLOBAL_FRAME_PID=""
LIDAR_PID=""
CV_PID=""
FUSION_PID=""
cleanup() {
    echo ""
    echo "=== Stopping MAVROS, global_frame, LiDAR, CV, and fusion pipelines ==="
    [ -n "$MAVROS_PID" ] && kill -TERM -"$MAVROS_PID" 2>/dev/null || true
    [ -n "$GLOBAL_FRAME_PID" ] && kill -TERM -"$GLOBAL_FRAME_PID" 2>/dev/null || true
    [ -n "$LIDAR_PID" ] && kill -TERM -"$LIDAR_PID" 2>/dev/null || true
    [ -n "$CV_PID" ]   && kill -TERM -"$CV_PID" 2>/dev/null || true
    [ -n "$FUSION_PID" ] && kill -TERM -"$FUSION_PID" 2>/dev/null || true
    sleep 1
    # Kill by name so every pipeline process is gone (ros2 launch spawns nodes separately)
    _KILL="ros2 launch|mavros|global_frame|boat_state_node|detection_to_global|v4l2_camera|v4l2_camera_node|camera0_node|camera1_node|camera2_node|cv_ros_nodes|vision_preprocessing|vision_inference|vision_combiner|maritime_distance|vision_lidar_fusion|task4_supply_processor|indicator_buoy_processor|maritime_distance_estimator|pointcloud_filters|unitree_lidar|lidar_range_filter|buoy_detector|buoy_tracker"
    pkill -f "$_KILL" 2>/dev/null || true
    sleep 1
    pkill -9 -f "$_KILL" 2>/dev/null || true
    echo "Done."
    exit 130
}
trap cleanup INT TERM

# Launch MAVROS if Pixhawk serial exists (or FCU_URL is e.g. tcp for sim)
if [[ "$FCU_URL" != /dev/* ]] || [ -e "${FCU_URL%%:*}" ]; then
  echo "=== Launching MAVROS (Pixhawk at ${FCU_URL}) ==="
  ros2 launch mavros apm.launch fcu_url:="${FCU_URL}" &
  MAVROS_PID=$!
  sleep 1
else
  echo "=== Skipping MAVROS (no device at ${FCU_URL%%:*}) ==="
fi

echo "=== Launching global_frame (boat_state_node + detection_to_global_node) ==="
ros2 launch global_frame global_frame.launch.py &
GLOBAL_FRAME_PID=$!
sleep 1

echo "=== Launching LiDAR buoy pipeline (no RViz, /dev/ttyUSB0) ==="
ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false &
LIDAR_PID=$!

sleep 2
echo "=== Launching CV pipeline ==="
ros2 launch cv_ros_nodes launch_cv.py use_sim:=false resolution:=960,600 conf_threshold:=0.1 preprocess_fps:=5 inference_interval_front:=4 inference_interval_sides:=6 camera_devices:="${CAMERA_DEVICES}" &
CV_PID=$!

sleep 4
echo "=== Starting CV-LiDAR fusion ==="
ros2 run cv_lidar_fusion vision_lidar_fusion &
FUSION_PID=$!

echo "=== All pipelines started. MAVROS PID: $MAVROS_PID  GLOBAL_FRAME PID: $GLOBAL_FRAME_PID  LiDAR PID: $LIDAR_PID  CV PID: $CV_PID  FUSION PID: $FUSION_PID ==="
echo "Press Ctrl+C to stop all."
wait -n
cleanup