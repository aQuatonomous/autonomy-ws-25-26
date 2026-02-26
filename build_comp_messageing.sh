#!/bin/bash
# Build all packages in src/comp_messageing.
# Run from autonomy-ws-25-26 (workspace root).
# Note: global_detections_bridge needs mapping workspace (pointcloud_filters, global_frame).
#       Build mapping first: cd mapping && colcon build --symlink-install

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"
MAPPING_WS="${SCRIPT_DIR}/mapping"

echo "=== Building comp_messageing packages ==="
source /opt/ros/humble/setup.bash

# Allow overriding message_node in this workspace to avoid colcon underlay warning.
colcon build --symlink-install \
  --allow-overriding message_node \
  --packages-select \
  message_node_msgs \
  message_node \
  task_sequence_coordinator

# Immediately source the freshly built workspace so console scripts like
# 'task_sequence_coordinator' and 'patrol_boat_detector' are available.
source "${SCRIPT_DIR}/install/setup.bash"

echo "=== comp_messageing build finished and sourced ==="
