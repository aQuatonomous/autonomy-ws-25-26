#!/bin/bash
# Run Gazebo with waves and course models (no boat, no SITL)
# Single terminal; useful for testing the wave sim before full boat simulation.
# See README.md for quick start and SIMULATION.md for full boat setup.

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." >/dev/null 2>&1 && pwd)"
ASV_INSTALL="${WS_ROOT}/src/asv_wave_sim/install"

echo "========================================="
echo " Gazebo Standalone Test (asv_wave_sim)"
echo "========================================="
echo

# Check if asv_wave_sim is built
if [ ! -f "${ASV_INSTALL}/setup.bash" ]; then
    echo "ERROR: asv_wave_sim not built yet!"
    echo
    echo "Build it first:"
    echo "  cd ${WS_ROOT}/src/asv_wave_sim"
    echo "  sudo apt install -y libcgal-dev libfftw3-dev"
    echo "  colcon build --symlink-install --merge-install --cmake-args \\"
    echo "    -DCMAKE_BUILD_TYPE=RelWithDebInfo \\"
    echo "    -DBUILD_TESTING=ON \\"
    echo "    -DCMAKE_CXX_STANDARD=17"
    echo
    exit 1
fi

# Check external dependencies (optional for course models test)
echo "Checking external dependencies..."
MISSING_DEPS=0

if [ ! -d "$HOME/SITL_Models/Gazebo/models" ]; then
    echo "  WARNING: ~/SITL_Models/Gazebo/models not found"
    echo "           (Boat model 'ourboat' will not load; this is OK for testing course models)"
    MISSING_DEPS=1
fi

if [ ! -d "$HOME/ardupilot_gazebo/build" ]; then
    echo "  WARNING: ~/ardupilot_gazebo/build not found"
    echo "           (ArduPilotPlugin will not load; this is OK for testing course models)"
    MISSING_DEPS=1
fi

if [ $MISSING_DEPS -eq 1 ]; then
    echo
    echo "  You can still test Gazebo with water + course models (buoys, gates, etc.)"
    echo "  See EXTERNAL_SETUP.md for how to set up the full boat simulation."
    echo
fi

# Source ROS 2 Jazzy
echo "Sourcing ROS 2 Jazzy..."
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ERROR: ROS 2 Jazzy not installed at /opt/ros/jazzy/"
    echo "Install it first: sudo apt install ros-jazzy-desktop"
    exit 1
fi
source /opt/ros/jazzy/setup.bash

# Source asv_wave_sim
echo "Sourcing asv_wave_sim install..."
export COLCON_CURRENT_PREFIX="${ASV_INSTALL}"
source "${ASV_INSTALL}/setup.bash"

# Set environment variables
echo "Setting Gazebo environment variables..."
export GZ_VERSION=harmonic

# LD_LIBRARY_PATH: Add asv_wave_sim libs (waves/hydro plugins)
# On x86 skip /usr/lib/aarch64-linux-gnu/nvidia (Jetson only)
if [ -d "/usr/lib/aarch64-linux-gnu/nvidia" ]; then
    export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu/nvidia:${ASV_INSTALL}/lib:$LD_LIBRARY_PATH"
else
    export LD_LIBRARY_PATH="${ASV_INSTALL}/lib:$LD_LIBRARY_PATH"
fi

# GZ_SIM_SYSTEM_PLUGIN_PATH: asv_wave_sim plugins (waves, hydro) first, then ArduPilotPlugin if present
_SAVED_GZ_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
GZ_SIM_SYSTEM_PLUGIN_PATH="${ASV_INSTALL}/lib"
if [ -d "$HOME/ardupilot_gazebo/build" ]; then
    GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:$HOME/ardupilot_gazebo/build"
fi
[ -n "$_SAVED_GZ_PLUGIN_PATH" ] && GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:$_SAVED_GZ_PLUGIN_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH

# GZ_SIM_RESOURCE_PATH: Add model paths
GZ_SIM_RESOURCE_PATH="${WS_ROOT}/src/asv_wave_sim/gz-waves-models/models"
GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${WS_ROOT}/src/asv_wave_sim/gz-waves-models/world_models"

# Add external paths if they exist
if [ -d "$HOME/SITL_Models/Gazebo/models" ]; then
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:$HOME/SITL_Models/Gazebo/models"
fi
if [ -d "$HOME/SITL_Models/Gazebo/worlds" ]; then
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:$HOME/SITL_Models/Gazebo/worlds"
fi
if [ -d "$HOME/ardupilot_gazebo/models" ]; then
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:$HOME/ardupilot_gazebo/models"
fi
if [ -d "$HOME/ardupilot_gazebo/worlds" ]; then
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:$HOME/ardupilot_gazebo/worlds"
fi

# Always append any existing GZ_SIM_RESOURCE_PATH
GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH

echo
echo "Environment ready. Starting Gazebo..."
echo "World: aquatonomous_world.sdf"
echo
echo "Expected results:"
echo "  - Water surface with waves (from WavesModel plugin)"
echo "  - Buoys, gates, rope holds (from gz-waves-models)"
echo "  - If ~/SITL_Models exists: 'ourboat' model loads"
echo "  - If ~/SITL_Models missing: 'Unable to find uri[model://ourboat]' error (expected)"
echo
echo "Press Ctrl+C to stop Gazebo"
echo "========================================="
echo

# Change to worlds directory and run Gazebo
cd "${WS_ROOT}/src/asv_wave_sim/gz-waves-models/worlds"
exec gz sim -v 4 -r aquatonomous_world.sdf
