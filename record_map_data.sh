#!/bin/bash
# Record Map Data - Bags all topics needed for mapping and web server visualization
# Run this while your pipeline (comp_single_camera.sh or full comp.sh) is running

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"
PLANNING_WS="${SCRIPT_DIR}/planning"

# Generate timestamped bag name
BAG_NAME="map_data_$(date +%Y%m%d_%H%M%S)"

echo "=== Recording Map Data ==="
echo "📦 Bag name: ${BAG_NAME}"
echo "🎯 Topics: /boat_pose, /global_detections, /combined/detection_info_with_distance, /tracked_buoys, /mavros/setpoint_velocity/cmd_vel_unstamped, /planned_path, /curr_task"
echo ""

# Source same workspaces as comp scripts (message types for all recorded topics)
echo "Sourcing ROS2 and workspaces..."
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"
source "${PLANNING_WS}/install/setup.bash"
echo "✓ Workspaces sourced"
echo ""

# Quick check: are boat_pose and global_detections being published?
echo "Checking for required publishers (2 sec)..."
BOAT_PUB=$(timeout 2 ros2 topic info /boat_pose 2>/dev/null | grep -c "Publisher count: [1-9]" || true)
GLOBAL_PUB=$(timeout 2 ros2 topic info /global_detections 2>/dev/null | grep -c "Publisher count: [1-9]" || true)
if [ "$BOAT_PUB" = "0" ] || [ "$GLOBAL_PUB" = "0" ]; then
  echo "⚠️  WARNING: /boat_pose or /global_detections has no publisher!"
  echo "   The map visualizer needs both. Ensure:"
  echo "   - global_frame is running (boat_state_node + detection_to_global)"
  echo "   - GPS/MAVROS connected so boat_pose is published"
  echo "   - LiDAR or CV pipeline running so global_detections is published"
  echo ""
  read -p "Record anyway? (y/N) " -n 1 -r
  echo
  if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted. Start comp_single_camera_task3.sh or comp.sh first."
    exit 1
  fi
fi

echo "🔴 Recording... Press Ctrl+C to stop"
echo "💡 Make sure your pipeline is running (comp_single_camera.sh or comp.sh)"
echo ""

# Record the key topics for mapping, web visualization, and planning output
ros2 bag record \
  -o "${BAG_NAME}" \
  --compression-mode file \
  --compression-format zstd \
  /boat_pose \
  /global_detections \
  /combined/detection_info_with_distance \
  /tracked_buoys \
  /mavros/setpoint_velocity/cmd_vel_unstamped \
  /planned_path \
  /curr_task

echo ""
echo "✅ Recording stopped. Bag saved as: ${BAG_NAME}"
echo ""
echo "📋 Topics actually recorded:"
if [ -d "${BAG_NAME}" ]; then
  ros2 bag info "${BAG_NAME}" || echo "   (could not read bag info)"
else
  echo "   (bag directory not found)"
fi
echo ""
echo "To replay with web visualizer:"
echo "  1. In one terminal: ./run_visualizer.sh"
echo "  2. In another terminal: ./replay_map.sh ${BAG_NAME}"
echo "  3. Open http://localhost:8080"