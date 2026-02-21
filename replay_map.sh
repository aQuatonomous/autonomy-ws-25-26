#!/bin/bash
# Replay a map-data bag with the web visualizer.
# Usage: ./replay_map.sh [bag_name]
#   If no bag_name given, uses the most recent map_data_* folder.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"
PLANNING_WS="${SCRIPT_DIR}/planning"

# Find bag to play
if [ -n "$1" ]; then
  BAG="$1"
else
  # Most recent map_data_* by modification time
  BAG=$(ls -td "${SCRIPT_DIR}"/map_data_* 2>/dev/null | head -1)
fi

if [ -z "$BAG" ] || [ ! -d "$BAG" ]; then
  echo "❌ No bag found. Usage: ./replay_map.sh <bag_name>"
  echo "   Example: ./replay_map.sh map_data_20260220_163509"
  echo "   Or run without args to use the most recent map_data_* folder."
  exit 1
fi

echo "=== Map Replay ==="
echo "📦 Bag: $BAG"
echo ""

# Source same workspaces as comp/record (message types for bag topics)
source /opt/ros/humble/setup.bash
source "${MAPPING_WS}/install/setup.bash"
source "${CV_WS}/install/setup.bash"
source "${PLANNING_WS}/install/setup.bash"

# Check bag has required topics with messages (errors shown to terminal)
INFO=$(ros2 bag info "$BAG") || { echo "❌ Failed to read bag info (see above)"; exit 1; }
BOAT_COUNT=$(echo "$INFO" | grep "/boat_pose" | grep -oE "Count: [0-9]+" | grep -oE "[0-9]+" || echo "0")
GLOBAL_COUNT=$(echo "$INFO" | grep "/global_detections" | grep -oE "Count: [0-9]+" | grep -oE "[0-9]+" || echo "0")

if [ "$BOAT_COUNT" = "0" ] || [ "$GLOBAL_COUNT" = "0" ]; then
  echo "❌ This bag has NO data for the map visualizer:"
  echo "   /boat_pose:         $BOAT_COUNT messages"
  echo "   /global_detections: $GLOBAL_COUNT messages"
  echo ""
  echo "The visualizer needs both. Record again with the full pipeline running:"
  echo "  1. ./comp_single_camera_task3.sh (or comp.sh) – must include global_frame"
  echo "  2. Wait until boat_pose and global_detections are publishing (GPS connected)"
  echo "  3. ./record_map_data.sh"
  echo ""
  exit 1
fi

echo "✓ Bag has $BOAT_COUNT boat_pose and $GLOBAL_COUNT global_detections messages"
echo ""
echo "▶ Playing with --loop. Press Ctrl+C to stop."
echo ""
echo "In another terminal, run: ./run_visualizer.sh"
echo "Then open: http://localhost:8080"
echo ""

exec ros2 bag play "$BAG" --loop
