#!/bin/bash
# Top-level build script for the repo.
# Run from autonomy-ws-25-26 directory.
#
# This builds:
#   - Root ROS2 workspace (./src)
#   - Computer vision workspace (./computer_vision/src)
#
# Examples (extra args are passed to the **root** build):
#   ./build.sh
#   ./build.sh --packages-select message_node task_sequence_coordinator
#   ./build.sh --packages-up-to task_sequence_coordinator

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Building root ROS2 workspace (./src) ==="
echo

# Source ROS2 from a clean environment
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

# Build from the workspace root; only consider packages under ./src
cd "${SCRIPT_DIR}"
colcon build --symlink-install --base-paths src "$@"

echo
echo "✓ Root workspace build finished."

# Source the freshly built root workspace as an underlay for downstream workspaces
source "${SCRIPT_DIR}/install/setup.bash"

# Build computer_vision workspace if present
if [ -d "${SCRIPT_DIR}/computer_vision/src" ]; then
  echo
  echo "=== Building computer_vision workspace (./computer_vision/src) ==="
  echo
  cd "${SCRIPT_DIR}/computer_vision"
  colcon build --symlink-install --base-paths src
  echo
  echo "✓ computer_vision workspace build finished."
else
  echo
  echo "Skipping computer_vision build (no computer_vision/src directory found)."
fi

echo
echo "✓ All builds finished."
echo "Next:"
echo "  source install/setup.bash"
echo "  (and for CV nodes, also: cd computer_vision && source install/setup.bash)"
