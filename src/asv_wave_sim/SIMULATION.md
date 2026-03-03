# Full Boat Simulation Stack – Complete Guide

This document covers running the **complete boat simulation:** Gazebo Sim + ArduPilot SITL + MAVROS + ROS 2 bridges. The boat floats on water, is controlled by ArduPilot (Rover), and can be driven via MAVProxy or ROS 2.

**For a minimal test (Gazebo + waves only, no boat), see [README.md](README.md).**

---

## Table of Contents

1. [What This Is](#what-this-is)
2. [Architecture](#architecture)
3. [External Dependencies](#external-dependencies)
4. [How to Run](#how-to-run)
5. [How to Control the Boat](#how-to-control-the-boat)
6. [Running with Computer Vision](#running-with-computer-vision)
7. [Ports and Communication](#ports-and-communication)
8. [Issues We Hit and How We Fixed Them](#issues-we-hit-and-how-we-fixed-them)
9. [Troubleshooting](#troubleshooting)

---

## What This Is

The full simulation stack consists of:

- **Gazebo Sim (gz sim)** – Runs the aquatonomous boat world (`aquatonomous_world.sdf`) with water, buoys, gates, and the **ourboat** model.
- **ros_gz_bridge** – Bridges Gazebo topics (lidar, cameras) to ROS 2.
- **MAVROS** (ROS 2) – Talks to ArduPilot over MAVLink.
- **ArduPilot SITL** (Rover, JSON model) – Simulates the autopilot; receives sensor data from Gazebo and sends motor commands back.

**Goal:** Run the sim, connect MAVROS to SITL, arm the boat, and send velocity/position setpoints so the boat moves in Gazebo.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│  YOU                                                                             │
│  • MAVProxy (terminal): mode GUIDED, arm throttle, position/velocity              │
│  • ROS 2 (terminal): /mavros/*, setpoints, services                              │
└─────────────────────────────────────────────────────────────────────────────────┘
         │                                    │
         ▼                                    ▼
┌──────────────────────┐           ┌──────────────────────┐
│  MAVROS (ROS 2)      │           │  ArduPilot SITL       │
│  • fcu_url :=        │  TCP      │  • Listens 5760       │
│    tcp://localhost:  │◄─────────►│  • Rover, JSON model │
│    5760              │  5760     │  • Sends to 9002      │
└──────────────────────┘           └──────────┬───────────┘
         │                                    │ UDP (JSON FDM)
         │                                    │ pose/IMU ← → motor cmds
         │                                    ▼
         │                         ┌──────────────────────┐
         │                         │  ArduPilotPlugin      │
         │                         │  (in Gazebo, ourboat)│
         │                         │  • Listens 127.0.0.1: │
         │                         │    9002               │
         │                         │  • Drives thrusters   │
         │                         └──────────┬───────────┘
         │                                    │
         │                                    ▼
         │                         ┌──────────────────────┐
         │                         │  Gazebo Sim          │
         │                         │  • World + water     │
         │                         │  • ourboat model     │
         │                         │  • WavesModel,       │
         │                         │    Hydrodynamics     │
         │                         └──────────┬───────────┘
         │                                    │
         │                                    │ topics (lidar, cameras)
         ▼                                    ▼
┌──────────────────────┐           ┌──────────────────────┐
│  ros_gz_bridge       │◄─────────►│  Gazebo topics       │
│  • /laser_points     │           │  • /lidar_topic/     │
│  • /camera0..2/      │           │    points, camera*   │
│    image_raw          │           │                      │
└──────────────────────┘           └──────────────────────┘
```

**Summary:**

- **MAVROS ↔ SITL:** TCP on port **5760**. MAVROS is the ROS 2 interface to the autopilot.
- **SITL ↔ Gazebo:** UDP on port **9002**. The ArduPilotPlugin receives pose/IMU and sends motor commands; the plugin applies thrust so the boat moves.
- **Gazebo ↔ ROS 2:** ros_gz_bridge republishes Gazebo topics as ROS 2 (lidar, cameras).

Gazebo must be running **before** SITL starts so the plugin binds to 9002 when SITL connects.

---

## External Dependencies

The full boat simulation requires these components (they live outside the `autonomy-ws-25-26` repository):

| Component | Path | Purpose |
|-----------|------|---------|
| **SITL_Models** | `~/SITL_Models/Gazebo/models/ourboat` | Boat model (mesh, SDF, plugins) |
| **ardupilot_gazebo** | `~/ardupilot_gazebo/build/` | ArduPilotPlugin (connects SITL to Gazebo) |
| **ardupilot** | `~/ardupilot/` | ArduPilot SITL binary (simulated autopilot) |
| **bridge_ws** | `~/bridge_ws/` | ros_gz_bridge (Gazebo ↔ ROS 2 for Jazzy) |
| **MAVProxy** | `~/.local/bin/mavproxy.py` | MAVProxy (command-line ground station) |

### 1. SITL_Models (Boat Model)

**What:** The `ourboat` Gazebo model (mesh, SDF, sensors, thrusters, ArduPilotPlugin config).

**Where it should be:** `~/SITL_Models/Gazebo/models/ourboat/`

**Structure:**
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

### 2. ardupilot_gazebo (ArduPilotPlugin)

**What:** The ArduPilot Gazebo plugin that connects ArduPilot SITL to Gazebo physics.

**Where it should be:** `~/ardupilot_gazebo/build/libArduPilotPlugin.so`

**How to get it:**
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

**Important:** The plugin must be built under **your user** (not root, not another user).

### 3. ardupilot (SITL)

**What:** ArduPilot Software-In-The-Loop (SITL) – the simulated autopilot.

**Where it should be:** `~/ardupilot/` (can be a symlink)

**How to get it:**
```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf rover
```

**If the repo is under another user's directory:**
1. Symlink it to your home: `ln -s /home/other_user/ardupilot ~/ardupilot`
2. Add it as a Git safe directory: `git config --global --add safe.directory /home/other_user/ardupilot`

### 4. bridge_ws (ros_gz_bridge for Jazzy)

**What:** The `ros_gz_bridge` package that bridges Gazebo topics to ROS 2.

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

### 5. MAVProxy

**What:** Command-line ground station for ArduPilot.

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

---

## How to Run

### Option A: Single Command with tmux (Recommended)

The easiest way to start everything:

```bash
bash ~/autonomy-ws-25-26/src/asv_wave_sim/run_full_simulation_tmux.bash
```

This starts a tmux session with three windows:
- **Window 0:** Gazebo + 4 bridge panes (lidar, camera1, camera2, camera3)
- **Window 1:** MAVROS + shell
- **Window 2:** ArduPilot SITL + MAVProxy

Switch windows with `Ctrl+b` then `0`, `1`, or `2`.

### Option B: Manual Launch in Separate Terminals

For more control and easier debugging, run each component in its own terminal. **Start them in this order and wait between steps:**

#### Terminal 1 – Gazebo (wait for "Loaded plugins" message)

```bash
cd ~/autonomy-ws-25-26
source /opt/ros/jazzy/setup.bash
source src/asv_wave_sim/install/setup.bash

export GZ_VERSION=harmonic
export LD_LIBRARY_PATH="$HOME/autonomy-ws-25-26/src/asv_wave_sim/install/lib:$LD_LIBRARY_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="\
$HOME/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/models:\
$HOME/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/SITL_Models/Gazebo/models:\
$HOME/SITL_Models/Gazebo/worlds:\
$HOME/ardupilot_gazebo/models:\
$HOME/ardupilot_gazebo/worlds:\
${GZ_SIM_RESOURCE_PATH:-}"

cd ~/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/worlds
gz sim -v 4 -r aquatonomous_world.sdf
```

**Wait for Gazebo to fully load and show entities before proceeding.**

#### Terminal 2 – ros_gz_bridge (Lidar) [Start ~10s after Gazebo]

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /lidar_topic/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  --ros-args -r /lidar_topic/points:=/laser_points
```

#### Terminal 3 – ros_gz_bridge (Camera 1)

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera1_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera1_topic:=/camera0/image_raw
```

#### Terminal 4 – ros_gz_bridge (Camera 2)

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera2_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera2_topic:=/camera1/image_raw
```

#### Terminal 5 – ros_gz_bridge (Camera 3)

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera3_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera3_topic:=/camera2/image_raw
```

#### Terminal 6 – MAVROS [Start ~35s after Gazebo]

```bash
source /opt/ros/jazzy/setup.bash

until (echo >/dev/tcp/localhost/5760) 2>/dev/null; do 
  echo "Waiting for SITL on port 5760..."; 
  sleep 2; 
done

ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760
```

#### Terminal 7 – ArduPilot SITL + MAVProxy [Start ~40s after Gazebo]

```bash
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"

cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON \
  --console --map \
  --custom-location="51.566151,-4.034345,10.0,-135"
```

---

## How to Control the Boat

### In MAVProxy (after SITL is up)

For simulation, you must set the arming parameter **first**, then switch mode and arm:

```text
param set ARMING_REQUIRE 0
mode GUIDED
arm throttle
```

Then drive the boat:

```text
position 10 0 0     # 10 m north (NED: North, East, Down)
velocity 2 0 0      # 2 m/s north
position 5 5 0      # 5 m north, 5 m east
```

**Do NOT use `guided lat lon` in MAVProxy for Rover**—use `position` / `velocity` or send global setpoints via ROS 2/MAVROS.

### From ROS 2

Global position setpoint example:
```bash
ros2 topic pub /mavros/setpoint_position/global geographic_msgs/msg/GeoPoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {latitude: 51.5662, longitude: -4.0343, altitude: 0.0}}}" --once
```

Use MAVROS services for mode, arm, etc., as needed.

---

## Running with Computer Vision

The sim bridges publish **`/camera0/image_raw`**, **`/camera1/image_raw`**, **`/camera2/image_raw`** from Gazebo. You can run the computer vision pipeline on these topics without real cameras.

**Order:** Start the **simulation first**, then in a **second terminal** start the CV launch.

**Launch CV (sim-only):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/autonomy-ws-25-26/computer_vision/install/setup.bash
ros2 launch cv_ros_nodes launch_cv_sim.py
```

**On Jetson — use one camera to avoid GPU OOM:**
```bash
ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true
```

This runs **camera1 (center)** only. Combine with lower confidence to detect more sim buoys:
```bash
ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true conf_threshold:=0.15
```

**Viewing detections:** In RViz, add an **Image** display and set the topic to **`/camera1/detections`**.

For more details, see the computer vision documentation.

---

## Ports and Communication

| Connection | Port | Protocol | Description |
|------------|------|----------|-------------|
| MAVROS ↔ SITL | 5760 | TCP | MAVLink; MAVROS uses `fcu_url:=tcp://localhost:5760` |
| SITL ↔ ArduPilotPlugin | 9002 | UDP | JSON FDM: pose/IMU from Gazebo, motor commands from SITL |
| MAVProxy ↔ SITL | 5760 | TCP | MAVProxy's `--master tcp:127.0.0.1:5760` |

---

## Issues We Hit and How We Fixed Them

### 1. Boat fell through the water

**Symptom:** Boat spawned then fell straight down through the water.

**Cause:** WavesModel and Hydrodynamics plugins did not load.

**Fix:**
- Add the waves install lib path to `LD_LIBRARY_PATH`:
  ```bash
  export LD_LIBRARY_PATH=$HOME/autonomy-ws-25-26/src/asv_wave_sim/install/lib:$LD_LIBRARY_PATH
  ```
- In `~/SITL_Models/Gazebo/models/ourboat/model.sdf`, ensure Hydrodynamics plugin has:
  ```xml
  <enable>ourboat::base_link</enable>
  ```

### 2. Boat did not move in Gazebo

**Symptom:** MAVProxy and SITL worked, but the 3D boat in Gazebo stayed still.

**Cause:** ArduPilotPlugin was not loaded or not built for the current user.

**Fix:**
- Ensure `GZ_SIM_SYSTEM_PLUGIN_PATH` includes `$HOME/ardupilot_gazebo/build`.
- Clean rebuild of the plugin under your user:
  ```bash
  cd ~/ardupilot_gazebo && rm -rf build && mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
  ```
- Verify `libArduPilotPlugin.so` exists in `~/ardupilot_gazebo/build/`.

### 3. MAVProxy not found

**Symptom:** SITL pane showed "No such file or directory: 'mavproxy.py'".

**Cause:** When tmux runs the SITL command, `PATH` did not include `~/.local/bin`.

**Fix:** Set in the script:
```bash
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"
```

### 4. Model name mismatch (ourboat vs blueboat)

**Symptom:** Boat doesn't move or plugins don't apply.

**Cause:** SDF had wrong model name or mesh URIs.

**Fix:** In `~/SITL_Models/Gazebo/models/ourboat/model.sdf`:
- Set `<model name="ourboat">`.
- Replace all `models://blueboat/` with `models://ourboat/`.
- Set Hydrodynamics `<enable>ourboat::base_link</enable>`.

### 5. Pre-arm: "AP ARM gyros inconsistent", "need position estimate"

**Symptom:** `arm throttle` refused.

**Cause:** Rover arming checks were enabled.

**Fix:** In MAVProxy:
```text
param set ARMING_REQUIRE 0
mode GUIDED
arm throttle
```

### 6. Wrong MAVROS port (connection refused)

**Symptom:** MAVROS failed to connect.

**Cause:** Script or docs used `tcp://localhost:5763`; SITL listens on **5760**.

**Fix:** Use `fcu_url:=tcp://localhost:5760` everywhere.

### 7. SITL build fails: Permission denied on `.wafpickle` (Jetson)

**Symptom:** `PermissionError: [Errno 13] Permission denied: '.../ardupilot/build/sitl/.wafpickle-...'`

**Cause:** `ardupilot` was built with `sudo` or owned by another user.

**Fix:**
```bash
cd ~/ardupilot
sudo chown -R $USER:$USER .
rm -rf build/sitl
./waf configure --board sitl
./waf rover
```

### 8. Gazebo black or white screen / "could not connect to display" (Jetson)

**Symptom:** Gazebo window doesn't open or stays black/white.

**Cause:** No display, or EGL/GPU/ogre2 issues on Jetson.

**Fix:**
- **Run on the Jetson from a local graphical session** with a monitor attached.
- If you're at the Jetson but shell has no display: `export DISPLAY=:0`
- For black/white: try adding `--render-engine-gui ogre` to the `gz sim` line; ensure NVIDIA drivers are installed.

### 9. Gazebo shows "0 entities" (empty world)

**Symptom:** Gazebo window opens but shows 0 entities.

**Cause:** The world or its included models failed to load. `GZ_SIM_RESOURCE_PATH` is missing required model dirs.

**Fix:**
1. Ensure `GZ_SIM_RESOURCE_PATH` includes:
   - `.../gz-waves-models/models` and `.../gz-waves-models/world_models` (for `model://waves`, etc.)
   - `~/SITL_Models/Gazebo/models` (for `model://ourboat`)
   - `~/ardupilot_gazebo/models` and `~/ardupilot_gazebo/worlds`
2. Ensure directories exist on your machine.
3. Check Gazebo output for "could not find model" errors.

### 10. "DO SET MODE failed: ap: flight mode change failed" (GUIDED / arm)

**Symptom:** In MAVProxy, `mode GUIDED` and/or `arm throttle` fail.

**Cause:** ArduPilot is refusing the mode change, usually because it has no position estimate or the world failed to load.

**Fix:**
1. Fix "0 entities" first (see above).
2. In MAVProxy run: `param set ARMING_REQUIRE 0`, then `mode GUIDED` and `arm throttle`.

### 11. Gazebo "0 entities" + libEGL / Mesa / nvidia-drm warnings (Jetson)

**Symptom:** Gazebo shows 0 entities and libEGL / MESA warnings.

**Cause:** Display/rendering stack using Mesa instead of NVIDIA driver.

**Fix:**
1. Force NVIDIA DRM mode: `sudo modprobe nvidia-drm modeset=1`
2. Ensure `LD_LIBRARY_PATH` starts with `/usr/lib/aarch64-linux-gnu/nvidia`
3. Try: `export __GLX_VENDOR_LIBRARY_NAME=nvidia`

---

## Troubleshooting

| Symptom | What to check |
|---------|----------------|
| Boat falls through water | `LD_LIBRARY_PATH` includes `.../asv_wave_sim/install/lib`; Hydrodynamics `<enable>` is `ourboat::base_link` |
| Boat doesn't move in Gazebo | `GZ_SIM_SYSTEM_PLUGIN_PATH` includes `~/ardupilot_gazebo/build`; plugin rebuilt for current user; SITL starts after Gazebo (40s delay) |
| MAVProxy not found | Script exports `PATH` and `PYTHONPATH` for SITL; `~/.local/bin/mavproxy.py` exists |
| MAVROS connection refused | SITL listening on 5760; use `fcu_url:=tcp://localhost:5760` |
| Can't arm: position / gyros | For sim: `param set ARMING_REQUIRE 0` then `mode GUIDED` and `arm throttle` |
| Gazebo black or white screen | `LD_LIBRARY_PATH` for NVIDIA libs (Jetson); ensure NVIDIA drivers installed |
| SITL build: Permission denied .wafpickle | `cd ~/ardupilot && sudo chown -R $USER:$USER . && rm -rf build/sitl && ./waf configure --board sitl && ./waf rover` |
| Gazebo shows 0 entities | `GZ_SIM_RESOURCE_PATH` must include all model dirs; check Gazebo pane for "could not find model" errors |
| Gazebo 0 entities + libEGL / Mesa / nvidia-drm | Run `sudo modprobe nvidia-drm modeset=1`; ensure `LD_LIBRARY_PATH` has NVIDIA libs first |
| DO SET MODE failed / stuck on MANUAL | **Set the param first:** `param set ARMING_REQUIRE 0`, then `mode GUIDED`, then `arm throttle` |

---

## Summary

The full boat simulation is ready to run. Choose one of these two approaches:

1. **Quick start (tmux):** `bash ~/autonomy-ws-25-26/src/asv_wave_sim/run_full_simulation_tmux.bash`
2. **Manual (separate terminals):** Follow the "How to Run" section above.

With everything working, the boat floats, arms, and moves in Gazebo under MAVProxy and ROS 2 control.
