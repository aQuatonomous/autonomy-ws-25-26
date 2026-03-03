# External Dependencies Setup Guide

This document describes the external dependencies needed for the **full** boat simulation stack. These live outside the `autonomy-ws-25-26` repository.

---

## Overview

The simulation has two levels:

1. **Minimal (course models only):** Water + buoys + gates  
   → Requires only `asv_wave_sim` (in this repo) to be built

2. **Full (boat + autopilot):** Everything above + boat model + ArduPilot SITL + MAVROS  
   → Requires external repos and models

---

## External Dependencies

| Component | Path | Purpose | Required for |
|-----------|------|---------|--------------|
| **SITL_Models** | `~/SITL_Models/Gazebo/models/ourboat` | Boat model (mesh, SDF, plugins) | Full stack |
| **ardupilot_gazebo** | `~/ardupilot_gazebo/build/` | ArduPilotPlugin (connects SITL to Gazebo) | Full stack |
| **ardupilot** | `~/ardupilot/` | ArduPilot SITL binary (simulated autopilot) | Full stack |
| **bridge_ws** | `~/bridge_ws/` | ros_gz_bridge (Gazebo ↔ ROS 2 for Jazzy) | Full stack (for camera/lidar topics in ROS 2) |
| **MAVProxy** | `~/.local/bin/mavproxy.py` | MAVProxy (command-line ground station) | Full stack |

---

## 1. SITL_Models (Boat Model)

**What:** The `ourboat` Gazebo model (mesh, SDF, sensors, thrusters, ArduPilotPlugin config).

**Where it should be:** `~/SITL_Models/Gazebo/models/ourboat/`

**How to get it:**

This is typically cloned from a separate repository or provided by your team. The structure should look like:

```
~/SITL_Models/
└── Gazebo/
    ├── models/
    │   └── ourboat/
    │       ├── model.sdf       # Main model definition
    │       ├── model.config    # Gazebo model metadata
    │       └── meshes/         # 3D meshes (DAE, STL, etc.)
    └── worlds/
        └── (optional world files)
```

**Check if you have it:**

```bash
ls -la ~/SITL_Models/Gazebo/models/ourboat/
```

**If you don't have it:** Ask your team for the SITL_Models repository or tarball. Alternatively, check if it's in the old `simulations/` folder docs or a separate repo.

---

## 2. ardupilot_gazebo (ArduPilotPlugin)

**What:** The ArduPilot Gazebo plugin that connects ArduPilot SITL (autopilot) to Gazebo (physics). It receives pose/IMU from Gazebo and sends motor commands from SITL.

**Where it should be:** `~/ardupilot_gazebo/build/libArduPilotPlugin.so`

**How to get it:**

Clone and build:

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

**Verify:**

```bash
ls -la ~/ardupilot_gazebo/build/libArduPilotPlugin.so
```

**Note:** The plugin must be built under **your user** (not root, not another user). If it was built under a different user, do a clean rebuild:

```bash
cd ~/ardupilot_gazebo
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

---

## 3. ardupilot (SITL)

**What:** ArduPilot Software-In-The-Loop (SITL) – the simulated autopilot that runs Rover firmware.

**Where it should be:** `~/ardupilot/` (can be a symlink)

**How to get it:**

Clone and build:

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf rover
```

**Verify:**

```bash
ls -la ~/ardupilot/Tools/autotest/sim_vehicle.py
```

**If the repo is under another user's directory:**

1. Symlink it to your home:
   ```bash
   ln -s /home/other_user/ardupilot ~/ardupilot
   ```

2. Add it as a Git safe directory:
   ```bash
   git config --global --add safe.directory /home/other_user/ardupilot
   ```

3. Rebuild SITL under your user (if needed):
   ```bash
   cd ~/ardupilot
   sudo chown -R $USER:$USER .
   rm -rf build/sitl
   ./waf configure --board sitl
   ./waf rover
   ```

---

## 4. bridge_ws (ros_gz_bridge for Jazzy)

**What:** The `ros_gz_bridge` package that bridges Gazebo topics to ROS 2. This is needed to get camera and lidar data into ROS 2.

**Where it should be:** `~/bridge_ws/install/`

**How to get it:**

```bash
mkdir -p ~/bridge_ws/src
cd ~/bridge_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b ros2

cd ~/bridge_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

**Verify:**

```bash
source ~/bridge_ws/install/setup.bash
ros2 pkg list | grep ros_gz_bridge
```

You should see `ros_gz_bridge` listed.

---

## 5. MAVProxy

**What:** MAVProxy is a command-line ground station for ArduPilot. It's used to control the simulated boat (arm, set mode, send position/velocity commands).

**Where it should be:** `~/.local/bin/mavproxy.py`

**How to get it:**

```bash
pip install --user MAVProxy
```

**Verify:**

```bash
ls -la ~/.local/bin/mavproxy.py
mavproxy.py --version
```

**Ensure `~/.local/bin` is in your PATH:**

```bash
export PATH="$HOME/.local/bin:$PATH"
```

Add this to your `~/.bashrc` if not already there.

---

## Quick Check Script

Run this to see what you have:

```bash
echo "Checking external dependencies..."
echo

echo "1. SITL_Models (boat model):"
if [ -d "$HOME/SITL_Models/Gazebo/models/ourboat" ]; then
    echo "   ✓ Found at ~/SITL_Models/Gazebo/models/ourboat"
else
    echo "   ✗ Not found at ~/SITL_Models/Gazebo/models/ourboat"
fi

echo "2. ardupilot_gazebo (plugin):"
if [ -f "$HOME/ardupilot_gazebo/build/libArduPilotPlugin.so" ]; then
    echo "   ✓ Found at ~/ardupilot_gazebo/build/libArduPilotPlugin.so"
else
    echo "   ✗ Not found at ~/ardupilot_gazebo/build/libArduPilotPlugin.so"
fi

echo "3. ardupilot (SITL):"
if [ -f "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" ]; then
    echo "   ✓ Found at ~/ardupilot/Tools/autotest/sim_vehicle.py"
else
    echo "   ✗ Not found at ~/ardupilot/Tools/autotest/sim_vehicle.py"
fi

echo "4. bridge_ws (ros_gz_bridge):"
if [ -d "$HOME/bridge_ws/install" ]; then
    echo "   ✓ Found at ~/bridge_ws/install"
else
    echo "   ✗ Not found at ~/bridge_ws/install"
fi

echo "5. MAVProxy:"
if [ -f "$HOME/.local/bin/mavproxy.py" ]; then
    echo "   ✓ Found at ~/.local/bin/mavproxy.py"
else
    echo "   ✗ Not found at ~/.local/bin/mavproxy.py"
fi

echo
echo "See EXTERNAL_SETUP.md for setup instructions."
```

---

## What You Can Test Without These

**With only `asv_wave_sim` built:**

You can test Gazebo with water + course models (buoys, gates, rope holds):

```bash
bash ~/autonomy-ws-25-26/src/asv_wave_sim/test_gazebo_standalone.sh
```

Gazebo will open, show water and course models, but report:
- "Unable to find uri[model://ourboat]" (expected; no boat)

**With `SITL_Models` + `ardupilot_gazebo` + everything else:**

You get the full stack: boat floats, SITL connects, MAVROS works, you can arm and drive the boat.

---

## Summary

| Dependency | Minimal Test | Full Stack |
|------------|--------------|------------|
| asv_wave_sim (this repo) | ✓ Required | ✓ Required |
| SITL_Models (boat) | Not needed | ✓ Required |
| ardupilot_gazebo | Not needed | ✓ Required |
| ardupilot (SITL) | Not needed | ✓ Required |
| bridge_ws | Not needed | ✓ Required |
| MAVProxy | Not needed | ✓ Required |

Start with the minimal test (`test_gazebo_standalone.sh`) to verify the wave sim works, then add the external dependencies for the full boat simulation.
