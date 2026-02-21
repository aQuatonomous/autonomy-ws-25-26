#!/bin/bash
# Build script for all ROS2 workspaces
# Run from autonomy-ws-25-26 directory

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPPING_WS="${SCRIPT_DIR}/mapping"
CV_WS="${SCRIPT_DIR}/computer_vision"
PLANNING_WS="${SCRIPT_DIR}/planning"
WEB_SERVER_WS="${SCRIPT_DIR}/web_server_map"

echo "=== Building All ROS2 Workspaces ==="
echo

#source ROS2
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
# Build mapping workspace
echo "=== Building mapping workspace ==="
cd "${MAPPING_WS}"
colcon build --symlink-install
echo "✓ Mapping workspace built successfully"
echo

# Build planning workspace (needs mapping for global_frame dependency)
echo "=== Building planning workspace ==="
source "${MAPPING_WS}/install/setup.bash"
cd "${PLANNING_WS}"
if [ -d "Global_Planner" ]; then
    colcon build --symlink-install --paths Global_Planner
    echo "✓ Planning workspace built successfully"
else
    echo "⚠ Global_Planner not found in planning directory"
fi
echo

# Build web server workspace
echo "=== Building web server workspace ==="
cd "${WEB_SERVER_WS}"
# Source mapping install for global_frame message types
source "${MAPPING_WS}/install/setup.bash"
colcon build --symlink-install
echo "✓ Web server workspace built successfully"
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
echo "✓ Planning workspace:        ${PLANNING_WS}"
echo "✓ Web server workspace:     ${WEB_SERVER_WS}"
echo
echo "All workspaces built successfully!"
echo "You can now run: ./comp.sh"
