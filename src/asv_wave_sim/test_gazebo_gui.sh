#!/bin/bash
# Test Gazebo GUI launch separately
# Run this first to verify Gazebo window opens

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." >/dev/null 2>&1 && pwd)"
ASV_INSTALL="${WS_ROOT}/src/asv_wave_sim/install"

echo "========================================="
echo " Test 1: Gazebo GUI Launch"
echo "========================================="

# Source ROS and asv_wave_sim
source /opt/ros/jazzy/setup.bash
export COLCON_CURRENT_PREFIX="${ASV_INSTALL}"
source "${ASV_INSTALL}/setup.bash"

# Set environment
export GZ_VERSION=harmonic
export LD_LIBRARY_PATH="${ASV_INSTALL}/lib:$LD_LIBRARY_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ASV_INSTALL}/lib:$HOME/ardupilot_gazebo/build"
export GZ_SIM_RESOURCE_PATH="${WS_ROOT}/src/asv_wave_sim/gz-waves-models/models:${WS_ROOT}/src/asv_wave_sim/gz-waves-models/world_models:$HOME/SITL_Models/Gazebo/models:$HOME/SITL_Models/Gazebo/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds"

echo "Starting Gazebo GUI..."
echo "Window should open showing boat, water, and course..."
echo ""

cd "${WS_ROOT}/src/asv_wave_sim/gz-waves-models/worlds"

# Fix GZ_CONFIG_PATH so 'gz sim' command is available
# ROS 2 sets GZ_CONFIG_PATH to only ROS vendor directories, we need system paths too
export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"

gz sim -v 2 -r aquatonomous_world.sdf
