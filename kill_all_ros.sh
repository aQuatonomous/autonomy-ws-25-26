#!/bin/bash
# Comprehensive ROS process killer. Run from autonomy-ws-25-26 root.
# Usage: ./kill_all_ros.sh  (all output to this terminal)

echo "=== Killing all ROS/autonomy processes ==="

# Kill by process name patterns (comprehensive list)
PATTERNS=(
    "ros2"
    "mavros"
    "unitree_lidar"
    "lidar_range_filter"
    "buoy_detector"
    "buoy_tracker"
    "buoy_visualizer"
    "tracked_buoy_visualizer"
    "camera.*_node"
    "v4l2_camera"
    "vision_preprocessing"
    "vision_inference"
    "vision_combiner"
    "cv_ros_nodes"
    "vision_lidar_fusion"
    "detection_to_global"
    "boat_state_node"
    "global_frame"
    "global_planner_node"
    "rqt"
)

for pattern in "${PATTERNS[@]}"; do
    echo "Killing processes matching: $pattern"
    pkill -f "$pattern" 2>/dev/null || true
    sleep 0.5
    pkill -9 -f "$pattern" 2>/dev/null || true
done

echo "=== Waiting for processes to terminate ==="
sleep 2

echo "=== Force killing any remaining ROS processes ==="
pkill -9 -f "ros2|mavros|unitree|lidar|buoy|camera|vision|cv_|detection|rqt" 2>/dev/null || true

echo "=== Stopping ROS2 daemon ==="
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 daemon stop 2>/dev/null || true

echo "=== Cleanup complete ==="
echo "Remaining topics:"
ros2 topic list 2>/dev/null || echo "No ROS2 daemon running"