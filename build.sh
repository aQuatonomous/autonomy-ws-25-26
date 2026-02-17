#!/bin/bash
# Build script for all ROS2 workspaces
# Run from autonomy-ws-25-26 directory

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"

echo "=== Building All ROS2 Workspaces ==="
echo

# Source ROS2
echo "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

# Build mapping workspace
echo "=== Building mapping workspace ==="
cd "${MAPPING_WS}"
colcon build --symlink-install
echo "✓ Mapping workspace built successfully"
echo

# Build computer vision workspace  
echo "=== Building computer vision workspace ==="
cd "${CV_WS}"
colcon build --symlink-install
echo "✓ Computer vision workspace built successfully"
echo

echo "=== Build Summary ==="
echo "✓ Mapping workspace:        ${MAPPING_WS}"
echo "✓ Computer vision workspace: ${CV_WS}"
echo
echo "Both workspaces built successfully!"
echo "You can now run: ./comp.sh"