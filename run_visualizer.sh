#!/bin/bash
# Web Server Map Visualizer - Single command launcher
# Run from autonomy-ws-25-26 root. Opens http://localhost:8080

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_SERVER_WS="${SCRIPT_DIR}/web_server_map"
MAPPING_WS="${SCRIPT_DIR}/mapping"

echo "=== Starting Web Server Map Visualizer ==="
echo "Sourcing ROS2 and workspaces..."

# Source ROS2 and required workspaces
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"    # for global_frame messages
source "${WEB_SERVER_WS}/install/setup.bash"  # for web_server_map

echo "✓ Workspaces sourced"
echo "🌐 Starting web server at http://localhost:8080"
echo "📡 Listening for topics: /boat_pose, /global_detections"
echo ""
echo "To use with rosbag playback:"
echo "  1. Keep this running"
echo "  2. In another terminal: source mapping/install/setup.bash && ros2 bag play your_bag_file"
echo "  3. Open http://localhost:8080 in browser"
echo ""
echo "Note: Bag must contain /boat_pose and /global_detections. Record with: ./record_map_data.sh"
echo ""
echo "Press Ctrl+C to stop..."

# Run the visualizer (use python -m so it works even when ros2 run doesn't find the executable)
exec python3 -m web_server_map.map_visualizer_node
