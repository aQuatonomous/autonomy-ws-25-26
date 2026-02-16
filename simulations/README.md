# Boat Simulation Stack – Full Documentation

This folder holds the **canonical simulation setup** for the autonomy boat: Gazebo Sim + ArduPilot SITL + MAVROS + ROS 2 bridges. The boat floats on water, is controlled by ArduPilot (Rover), and can be driven via MAVProxy or ROS 2.

---

## Table of Contents

1. [What This Is](#what-this-is)
2. [Architecture: How Everything Is Connected](#architecture-how-everything-is-connected)
3. [What Is Working Right Now](#what-is-working-right-now)
4. [Key Files and Paths](#key-files-and-paths)
5. [User and Directory Setup (Ethan vs Lorenzo)](#user-and-directory-setup-ethan-vs-lorenzo)
6. [How to Run the Simulation](#how-to-run-the-simulation)
7. [Quick command reference](#quick-command-reference)
8. [Tmux Layout and Navigation](#tmux-layout-and-navigation)
9. [How to Control the Boat](#how-to-control-the-boat)
10. [Running computer vision with the simulation](#running-computer-vision-with-the-simulation)
11. [Ports and Communication](#ports-and-communication)
12. [Issues We Hit and How We Fixed Them](#issues-we-hit-and-how-we-fixed-them)
13. [One-Time Setup and Dependencies](#one-time-setup-and-dependencies)
14. [Troubleshooting](#troubleshooting)

---

## What This Is

- **Gazebo Sim (gz sim)** runs the **Aquatonomous** boat world (`aquatonomous_world.sdf` in `simulations/worlds/`) with water, buoys, gates, and the **ourboat** model.
- **ros_gz_bridge** bridges Gazebo topics to ROS 2 (lidar + 3 cameras).
- **MAVROS** (ROS 2) talks to ArduPilot over MAVLink.
- **ArduPilot SITL** (Rover, JSON model) simulates the autopilot; it receives sensor data from Gazebo and sends motor commands back so the boat moves in the world.

**Goal:** Run the sim, connect MAVROS to SITL, arm the boat, and send velocity/position setpoints so the boat moves in Gazebo and you can develop autonomy (CV, planning, etc.) on top.

---

## Architecture: How Everything Is Connected

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│  YOU                                                                             │
│  • MAVProxy (tmux window 2): mode GUIDED, arm throttle, position/velocity         │
│  • ROS 2 (e.g. shell panes): /mavros/*, setpoints, services                      │
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
- **SITL ↔ Gazebo:** UDP on port **9002**. The ArduPilotPlugin inside the ourboat model receives pose/IMU from Gazebo and sends motor commands from SITL; the plugin applies thrust so the boat moves.
- **Gazebo ↔ ROS 2:** ros_gz_bridge subscribes to Gazebo topics and republishes as ROS 2 (lidar, cameras).

Gazebo must be running **before** SITL starts so the plugin is bound to 9002 when SITL connects. The script enforces this with a 40 s delay before starting SITL.

---

## What Is Working Right Now

- **Gazebo** loads the world `aquatonomous_world.sdf` (in `simulations/worlds/`) with water (waves model), buoys, gates, and the ourboat model.
- **Water physics:** WavesModel and Hydrodynamics plugins load; the boat **floats** on the water and does not fall through (requires correct `LD_LIBRARY_PATH` and model name; see fixes below).
- **ArduPilotPlugin** loads; SITL connects to it on 9002; the boat **moves in Gazebo** when you send position/velocity commands.
- **MAVProxy** connects to SITL; you can run `mode GUIDED`, `arm throttle`, `position X Y Z`, `velocity X Y Z` and see the boat move.
- **Arming:** With `ARMING_REQUIRE 0` the boat arms without a GPS position estimate (no “need position estimate” block).
- **ROS 2 bridges:** Lidar and three cameras are bridged to ROS 2 (`/laser_points`, `/camera0/image_raw`, etc.).
- **MAVROS** connects to SITL and exposes ROS 2 topics/services for setpoints, mode, arm, etc.

---

## Key Files and Paths

| What | Path |
|------|------|
| **This folder** | `autonomy-ws-25-26/simulations/` |
| **Launch script** | `simulations/simulation_full.bash` (or `~/simulation_full.bash` if you copy it) |
| **World (map)** | `autonomy-ws-25-26/simulations/worlds/aquatonomous_world.sdf` |
| **Wave/course models** | `.../gz-waves-models/models/` and `.../gz-waves-models/world_models/` (waves, RedSurMark, buoys, etc.; must be in `GZ_SIM_RESOURCE_PATH`) |
| **Boat model** | `~/SITL_Models/Gazebo/models/ourboat/` (model.sdf, model.config, meshes) |
| **ArduPilot plugin build** | `~/ardupilot_gazebo/build/` (must contain `libArduPilotPlugin.so`) |
| **ArduPilot repo (SITL)** | `~/ardupilot` (often a symlink; see [User and Directory Setup](#user-and-directory-setup-ethan-vs-lorenzo)) |
| **SITL entrypoint** | `~/ardupilot/Tools/autotest/sim_vehicle.py` (must run from `Tools/autotest`) |
| **Waves/hydro libs** | `~/autonomy-ws-25-26/src/asv_wave_sim/install/lib/` (for Hydrodynamics/WavesModel) |
| **Bridge workspace** | `~/bridge_ws/` (ros_gz_bridge) |

All paths in the script use `$HOME` so the same script works for different users as long as the layout under `~` is the same.

---

## User and Directory Setup (Ethan vs Lorenzo)

The stack was used on a machine where:

- **ArduPilot** lived under another user’s home (e.g. `/home/ethan/ardupilot`).
- **Your user** (e.g. `lorenzo`) runs the simulation and has their own home (`/home/lorenzo`).

To avoid “wrong user” and “wrong path” issues:

1. **Symlink ArduPilot into your home**  
   So the script only uses `~/ardupilot`:
   ```bash
   ln -s /home/ethan/ardupilot ~/ardupilot
   ```
   Then the script’s `cd ~/ardupilot/Tools/autotest` and `sim_vehicle.py` work regardless of which user owns `/home/ethan/ardupilot`.

2. **Git “dubious ownership”**  
   If ArduPilot is in another user’s directory, Git may refuse to run:
   ```bash
   git config --global --add safe.directory /home/ethan/ardupilot
   ```
   Do this once per machine.

3. **ArduPilot Gazebo plugin build**  
   The plugin must be **built in your own home** so that:
   - `~/ardupilot_gazebo/build/libArduPilotPlugin.so` exists and is loadable by Gazebo.
   - There are no hardcoded paths to another user’s home in the build.  
   If the build was originally done under e.g. `ethan`, do a clean rebuild under your user:
   ```bash
   cd ~/ardupilot_gazebo
   rm -rf build && mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
   make -j4
   ```
   Then ensure `GZ_SIM_SYSTEM_PLUGIN_PATH` includes `$HOME/ardupilot_gazebo/build` (the script does this).

4. **SITL_Models and ardupilot_gazebo**  
   These can live in your home (`~/SITL_Models`, `~/ardupilot_gazebo`). The script sets `GZ_SIM_RESOURCE_PATH` so Gazebo finds `model://ourboat` and related assets. No need for them to be under the same user as ArduPilot.

5. **MAVProxy**  
   Install under your user so the script finds it:
   ```bash
   pip install --user MAVProxy
   ```
   Script sets `PATH="$HOME/.local/bin:$PATH"` and `PYTHONPATH` so `mavproxy.py` is found when SITL starts.

**Summary:** Use `~/ardupilot` (symlink if needed), `~/ardupilot_gazebo`, `~/SITL_Models`, and `~/.local/bin` for MAVProxy. Build the plugin under your user and add the repo as a Git safe directory if it lives under another user’s path.

---

## How to Run the Simulation

1. **From the repo (recommended):**
   ```bash
   cd ~/autonomy-ws-25-26
   bash simulations/simulation_full.bash
   ```

2. **From anywhere** (single canonical script in the repo):
   ```bash
   bash ~/autonomy-ws-25-26/simulations/simulation_full.bash
   ```
   Optionally symlink for a shorter command: `ln -s ~/autonomy-ws-25-26/simulations/simulation_full.bash ~/simulation_full.bash`

The script will:

- Kill any existing tmux session named `simfull`.
- Start a new session with three windows (Gazebo+bridges, MAVROS, ArduPilot).
- Attach you to the session (if you’re in a real terminal); you’ll land on the Gazebo window.

If you run the script from an environment that can’t attach (e.g. some IDEs), the session still starts in the background. Attach manually:

```bash
tmux attach -t simfull
```

**On the Jetson (or any machine with a physical display):** Run the script from a **local graphical session** — e.g. open a terminal on the Jetson desktop and run `./simulation_full.bash` there. Gazebo needs a real display: if you run over SSH without X11 forwarding (or from a headless/IDE terminal with no `DISPLAY`), the Gazebo GUI will crash with “could not connect to display” / “Qt platform plugin xcb” and you’ll get a white screen or no window. If you’re logged in at the Jetson with a monitor but your terminal has no `DISPLAY`, try `export DISPLAY=:0` then run the script.

---

## Quick command reference

Copy-paste friendly commands. Run from appropriate directories or adjust paths.

**1. Start the simulation (from repo root or simulations folder):**
```bash
cd ~/autonomy-ws-25-26
bash simulations/simulation_full.bash
```

**2. On Jetson only — if you see libEGL / Mesa / nvidia-drm or 0 entities, run before starting the sim:**
```bash
sudo modprobe nvidia-drm modeset=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

**3. In MAVProxy (tmux window 2, left pane) — control the boat (order matters):**
```text
param set ARMING_REQUIRE 0
mode GUIDED
arm throttle
position 10 0 0
```
(or `velocity 2 0 0` for constant velocity)

**4. ArduPilot SITL build permission error (Jetson / .wafpickle):**
```bash
cd ~/autonomy-ws-25-26/simulations
./fix_ardupilot_build_permissions.sh
```
Or by hand: `cd ~/ardupilot && sudo chown -R $USER:$USER . && rm -rf build/sitl && ./waf configure && ./waf rover`

**5. Run computer vision with the sim (in a second terminal, after sim is up):**
```bash
source /opt/ros/humble/setup.bash
source ~/autonomy-ws-25-26/computer_vision/install/setup.bash
ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true
```
Use **camera1 (center)** only to save GPU memory on Jetson. For more buoy detections in sim, add `conf_threshold:=0.15`. For **smoother FPS** in rqt (display at ~30 FPS, inference every 2nd frame), add `inference_interval:=2`. View detections in RViz: Image topic `/camera1/detections`.

**6. Attach to sim if it started in background:**
```bash
tmux attach -t simfull
```

---

## Tmux Layout and Navigation

| Window | Name      | Contents |
|--------|-----------|----------|
| **0**  | (default) | Left: Gazebo. Right: 4 panes (lidar bridge, camera1, camera2, camera3). |
| **1**  | mavros    | Left: MAVROS. Right: shell. |
| **2**  | ardupilot | Left: ArduPilot SITL + MAVProxy. Right: shell. |

- **Switch windows:** `Ctrl+b` then `0`, `1`, or `2`.
- **Switch panes (same window):** `Ctrl+b` then arrow keys, or `Ctrl+b o`.

All ArduPilot/MAVProxy commands are typed in **window 2, left pane**.

---

## How to Control the Boat

**In MAVProxy (window 2, left pane), after the SITL is up:**

For **simulation** you must set the arming parameter **first**, then switch mode and arm. If you run `mode GUIDED` before this, you will get **“AP: flight mode change failed”** and stay in MANUAL.

**1. One-time (each SITL session) — required for sim:**

```text
param set ARMING_REQUIRE 0
```

**2. Then switch to GUIDED and arm:**

```text
mode GUIDED
arm throttle
```

**3. Drive the boat:**

```text
position 10 0 0     # 10 m north (NED: North, East, Down)
velocity 2 0 0      # 2 m/s north
position 5 5 0      # 5 m north, 5 m east
```

- Use **`position X Y Z`** (meters in NED from home) or **`velocity X Y Z`** (m/s in NED).
- Do **not** use `guided lat lon` in MAVProxy for Rover—that syntax is for other vehicle types and can cause errors or crashes. Use `position` / `velocity` or send global setpoints via ROS 2/MAVROS.

**If you already tried `mode GUIDED` and it failed:** run **`param set ARMING_REQUIRE 0`** first, then **`mode GUIDED`** and **`arm throttle`** again.

**From ROS 2 (e.g. shell pane with ROS sourced):**

- Global position setpoint example:
  ```bash
  ros2 topic pub /mavros/setpoint_position/global geographic_msgs/msg/GeoPoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {latitude: 51.5662, longitude: -4.0343, altitude: 0.0}}}" --once
  ```
- Use MAVROS services for mode, arm, etc., as needed.

---

## Running computer vision with the simulation

The sim bridges publish **`/camera0/image_raw`**, **`/camera1/image_raw`**, **`/camera2/image_raw`** from Gazebo. You can run the computer vision pipeline (preprocess → inference → combiner) on these topics **without** real cameras.

**Order:** Start the **simulation first** (so the bridges are publishing), then in a **second terminal** start the CV launch.

**Launch CV (sim-only, no v4l2 camera nodes):**
```bash
source /opt/ros/humble/setup.bash
source ~/autonomy-ws-25-26/computer_vision/install/setup.bash
ros2 launch cv_ros_nodes launch_cv_sim.py
```

**On Jetson — use one camera to avoid GPU OOM (NvMapMemAlloc error 12 / inference crash):**
```bash
ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true
```
This runs **camera1 (center)** only. Combiner will show “1 active, 2 no data”. View detections: **`/camera1/detections`** (Image) and **`/camera1/detection_info`** or **`/combined/detection_info`** (JSON).

**Optional — lower confidence so more sim buoys are detected:**
```bash
ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true conf_threshold:=0.15
```
Default is `0.25`; try `0.1` if you still see no buoy boxes.

**Viewing detections:** In RViz, add an **Image** display and set the topic to **`/camera1/detections`** (or `/camera0/detections`, `/camera2/detections` when not using single_camera). The `detection_info` topics are JSON strings, not images.

**Alternative (main launch with flag):** `ros2 launch cv_ros_nodes launch_cv.py use_sim:=true` — skips v4l2 camera nodes so image_raw comes from the bridges. Prefer `launch_cv_sim.py` for sim.

**Build CV workspace once** so the sim launch and args are available:
```bash
cd ~/autonomy-ws-25-26/computer_vision
colcon build --packages-select cv_ros_nodes
source install/setup.bash
```

**If you see “3 no data” in the combiner:** You need to use `launch_cv_sim.py` or `launch_cv.py use_sim:=true` so the pipeline does not start v4l2 camera nodes; image_raw must come from the sim bridges. Start the sim first, then CV.

**Buoy detection in sim:** The YOLO model was trained on real (or different) imagery; sim buoys often look different, so you may get few or no detections. Lowering `conf_threshold` (e.g. `0.15` or `0.1`) can show more boxes. If you see any boxes at all, the pipeline is working.

**Flickering (Gazebo or image topics):** On Jetson, Gazebo + RViz + CV can overload the GPU. Close extra viewers or run with single_camera. Ensure only one CV pipeline subscribes to each `/cameraN/image_raw`.

**FPS and inference speed:** On Jetson, TensorRT inference is the bottleneck (~2–5 FPS depending on engine and GPU). To get **higher FPS and detections on every frame**:

- **Detections on every frame:** With `inference_interval:=2` or `3`, the node runs inference every 2nd or 3rd frame. By default **`draw_stale_boxes:=true`**: on non-inference frames it still draws the **last** detections (dimmer green, “prev” label) so you see boxes on every frame and display FPS stays high (~30). New detections appear at inference rate; boxes may lag slightly when the boat moves fast. Set `draw_stale_boxes:=false` if you prefer no boxes on skip frames.
- **Max inference rate (detections per second):** Use `inference_interval:=1` to run inference on **every** frame. That gives the highest detections/second and best accuracy, but display FPS will match inference FPS (~2–5 on Jetson). Good when you need fresh detections every frame and can accept lower FPS.
- **Faster inference (higher FPS):** Rebuild the TensorRT engine with **FP16** for roughly 2× speed with minimal accuracy loss. From the computer_vision repo (see `model_building_and_training/TENSORRT.md`):
  ```bash
  # Export ONNX first if needed, then build FP16 engine (replace path to trtexec if needed)
  cd ~/autonomy-ws-25-26/computer_vision
  python model_building_and_training/export_onnx.py model_building_and_training/aqua_main.pt
  /usr/src/tensorrt/bin/trtexec --onnx=model_building_and_training/aqua_main.onnx --saveEngine=cv_scripts/model.engine --fp16 --memPoolSize=workspace:4096 --skipInference
  ```
  Copy the new `model.engine` to where launch expects it (`engine_path`). INT8 is faster still but requires calibration.
- **Competition-like (max detections, best accuracy):** Use **`inference_interval:=1`** (default) so every frame is inferred—same as on the real boat. See [computer_vision/README.md](../computer_vision/README.md#roboat-competition-best-for-detections-and-accuracy) for the full competition recommendation (FP16 engine, conf_threshold, etc.).
- Example for smooth 30 Hz display with detections drawn every frame (debug only; fewer detection updates):
  ```bash
  ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true inference_interval:=2
  ```
  Preprocess and inference use **QoS depth 1** so only the latest image is processed (no backlog).

**Sim camera resolution:** The ourboat model cameras are set to **1920×1200** (same as real AR0234 cameras). So `/cameraN/image_raw` from the sim is 1920×1200; the CV pipeline letterboxes to 640×640 for inference, then outputs detections and images at **1920×1200**. This matches the real-camera flow.

**Detection consistency (weird or missing boxes):** The pipeline uses **letterbox** preprocessing (aspect ratio preserved, no stretch) for **any** input resolution (1920×1200 real or sim, 640×480, etc.). The model sees correct geometry; boxes are transformed back to the original image size. **Real 1920×1200 cameras are supported and improved** (no stretch). With `draw_stale_boxes:=true` (default), when `inference_interval` > 1 you still see boxes on every frame (last result reused); they may lag slightly when moving. If buoys still aren’t detected when close or when still, sim appearance may differ from training; try `conf_threshold:=0.1` and ensure the buoy is well in frame.

**RViz point cloud (lidar):** Set the transform frame to `ourboat/lidar_link/lidar` (or your boat model name).

**RAM:** The sim uses a lot of RAM; run one instance at a time. If it runs out of memory, try closing other heavy apps (e.g. VSCode/Cursor).

---

## Ports and Communication

| Connection           | Port  | Protocol | Description |
|----------------------|-------|----------|-------------|
| MAVROS ↔ SITL        | 5760  | TCP      | MAVLink; MAVROS uses `fcu_url:=tcp://localhost:5760`. |
| SITL ↔ ArduPilotPlugin | 9002 | UDP      | JSON FDM: pose/IMU from Gazebo, motor commands from SITL. |
| MAVProxy ↔ SITL      | 5760  | TCP      | MAVProxy’s `--master tcp:127.0.0.1:5760`. |

Do not use port 5763; nothing is bound there. If MAVROS fails to connect, ensure SITL has printed that it is listening on 5760 (window 2).

---

## Issues We Hit and How We Fixed Them

### 1. Boat fell through the water

- **Symptom:** Boat spawned then fell straight down through the water.
- **Cause:** WavesModel and Hydrodynamics plugins did not load. Gazebo couldn’t find `libgz-waves1.so.1` (and related libs), so there was no water surface or buoyancy.
- **Fix:**
  - Add the waves install lib path to `LD_LIBRARY_PATH` in the Gazebo command:
    ```bash
    export LD_LIBRARY_PATH=$HOME/autonomy-ws-25-26/src/asv_wave_sim/install/lib:$LD_LIBRARY_PATH
    ```
  - In `~/SITL_Models/Gazebo/models/ourboat/model.sdf`, the Hydrodynamics plugin had `<enable>blueboat::base_link</enable>`. The model name was renamed to `ourboat`, so the link is `ourboat::base_link`. Changed to:
    ```xml
    <enable>ourboat::base_link</enable>
    ```
- **Result:** WavesModel and Hydrodynamics load; boat floats and responds to thrust correctly.

### 2. Boat did not move in Gazebo (map moved, boat did not)

- **Symptom:** MAVProxy and SITL worked, map showed movement, but the 3D boat in Gazebo stayed still.
- **Cause:** ArduPilotPlugin was not loaded or not built for the current user. Plugin path was wrong or build was under another user (e.g. ethan) with wrong paths.
- **Fix:**
  - Ensure `GZ_SIM_SYSTEM_PLUGIN_PATH` includes `$HOME/ardupilot_gazebo/build`.
  - Clean rebuild of the plugin under the user running the sim:
    ```bash
    cd ~/ardupilot_gazebo && rm -rf build && mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
    ```
  - Verify `libArduPilotPlugin.so` exists in `~/ardupilot_gazebo/build/`.
- **Result:** Plugin loads; SITL connects to 9002; boat moves in Gazebo when you send position/velocity.

### 3. MAVProxy not found (“No such file or directory: 'mavproxy.py'”)

- **Symptom:** SITL pane showed error and SITL exited; MAVProxy was installed but not found.
- **Cause:** When tmux runs the SITL command, `PATH` did not include `~/.local/bin`, and/or Python didn’t find MAVProxy’s modules.
- **Fix:** In the script, for the ArduPilot window command, set:
  ```bash
  export PATH="$HOME/.local/bin:$PATH"
  export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"
  ```
  and run SITL with:
  ```bash
  python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON ...
  ```
- **Result:** MAVProxy starts correctly and connects to SITL.

### 4. Model name mismatch (ourboat vs blueboat)

- **Symptom:** Possible load or link errors; Hydrodynamics not applying; or boat doesn’t move because Thruster/ArduPilotPlugin use wrong topic namespace.
- **Cause:** Directory was `ourboat`, but the SDF had `<model name="blueboat">`, mesh URIs `models://blueboat/...`, or **Thruster/ArduPilotPlugin** used `<namespace>blueboat</namespace>` and `<cmd_topic>/model/blueboat/...</cmd_topic>` so commands go to the wrong topic.
- **Fix:** In `~/SITL_Models/Gazebo/models/ourboat/model.sdf`:
  - Set `<model name="ourboat">`.
  - Replace all `models://blueboat/` with `models://ourboat/`.
  - Set Hydrodynamics `<enable>ourboat::base_link</enable>` (as above).
  - Set Thruster `<namespace>ourboat</namespace>` and ArduPilotPlugin `<cmd_topic>/model/ourboat/joint/.../cmd_thrust</cmd_topic>` (not `blueboat`).
- **Result:** Gazebo and plugins consistently use `ourboat`; thrust commands reach the boat.

### 5. Pre-arm: “AP ARM gyros inconsistent”, “need position estimate”

- **Symptom:** `arm throttle` refused; ArduPilot asked for position estimate and reported gyro issues.
- **Cause:** Rover arming checks (including position) were enabled; in sim we may not have a full GPS/position pipeline.
- **Fix:** In MAVProxy:
  ```text
  param set ARMING_REQUIRE 0
  mode GUIDED
  arm throttle
  ```
- **Result:** Boat arms and accepts position/velocity commands in simulation.

### 6. Wrong MAVROS port (connection refused)

- **Symptom:** MAVROS failed to connect to the autopilot.
- **Cause:** Script or docs used `tcp://localhost:5763`; SITL listens on **5760**.
- **Fix:** Use `fcu_url:=tcp://localhost:5760` everywhere. The script was updated and MAVROS waits until port 5760 is open (with a minimum delay) before launching.
- **Result:** MAVROS connects reliably once SITL is up.

### 7. Git “dubious ownership” when building/running SITL

- **Symptom:** Git errors when ArduPilot repo is under another user’s home (e.g. `/home/ethan/ardupilot`).
- **Fix:** One-time:
  ```bash
  git config --global --add safe.directory /home/ethan/ardupilot
  ```
- **Result:** SITL and builds can run without Git blocking.

### 8. Gazebo black or white screen / “could not connect to display” (Jetson)

- **Symptom:** Gazebo window doesn’t open, or stays black/white; or you see **“qt.qpa.xcb: could not connect to display”** / “Could not load the Qt platform plugin” in the Gazebo pane.
- **Cause:** (1) No display: you ran the script from SSH without X11 forwarding, or from a terminal/IDE that has no `DISPLAY` set — the Gazebo GUI then exits immediately. (2) Or: EGL/GPU/ogre2 on Jetson (black or white window).
- **Fix:** (1) **Run on the Jetson from a local graphical session:** open a terminal on the Jetson desktop (with a monitor attached) and run `./simulation_full.bash` there so `DISPLAY` is set. If you’re at the Jetson but the shell has no display, run `export DISPLAY=:0` then the script. (2) For black/white only: script already sets `LD_LIBRARY_PATH=.../nvidia:...`; try adding `--render-engine-gui ogre` to the `gz sim` line; ensure NVIDIA drivers are installed.
- **Result:** Gazebo GUI starts and world becomes visible.

### 9. SITL build fails: Permission denied on `.wafpickle` (Jetson / multi-user)

- **Symptom:** ArduPilot SITL pane shows `PermissionError: [Errno 13] Permission denied: '.../ardupilot/build/sitl/.wafpickle-...'`; build reaches link stage then Waf aborts; SITL never starts, so MAVROS stays on “Waiting for SITL on 5760”.
- **Cause:** `ardupilot` or `build/sitl` was created or built with `sudo`, or files are owned by another user. Waf then cannot write its state file in `build/sitl/`.
- **Fix:** On the machine where `~/ardupilot` lives (e.g. the Jetson), reset ownership and do a clean SITL rebuild (do **not** use `sudo` for builds):

  ```bash
  cd ~/ardupilot
  sudo chown -R $USER:$USER .
  rm -rf build/sitl
  ./waf configure
  ./waf rover
  ```

  Or run the helper script from this repo (on that machine): `simulations/fix_ardupilot_build_permissions.sh`.
- **Result:** SITL builds and starts; MAVROS can connect to port 5760.

### 10. Gazebo shows “0 entities” (empty world)

- **Symptom:** Gazebo window opens but the Entities panel on the right shows **0 entities**; no boat, no water, nothing in the scene.
- **Cause:** The world or its included models failed to load. The world `aquatonomous_world.sdf` uses `model://waves`, `model://ourboat`, `model://RedSurMark`, etc. Gazebo looks these up in `GZ_SIM_RESOURCE_PATH`. If that variable is missing the **gz-waves-models** dirs, Gazebo finds nothing and loads 0 entities. Other causes: world file not found (wrong path on Jetson); missing `~/SITL_Models` or `~/ardupilot_gazebo`; workspace not at `~/autonomy-ws-25-26` on this machine.
- **Fix:**
  1. **Resource path (script fix):** The launch script must set `GZ_SIM_RESOURCE_PATH` to include the wave/course models: `.../gz-waves-models/models` and `.../gz-waves-models/world_models` (so `model://waves`, `model://RedSurMark`, etc. resolve), plus `~/SITL_Models/Gazebo/models` (for `model://ourboat`) and `~/ardupilot_gazebo/models` and `~/ardupilot_gazebo/worlds`. The current `simulation_full.bash` includes these; if you edited it, restore the full `GZ_SIM_RESOURCE_PATH` line.
  2. **Paths:** Ensure on the Jetson you have: `~/autonomy-ws-25-26/simulations/worlds/aquatonomous_world.sdf`, `gz-waves-models/models/`, `gz-waves-models/world_models/`, `~/SITL_Models/Gazebo/models`, `~/ardupilot_gazebo/models`. Same layout under `$HOME` as on the machine where the script was written.
  3. **Gazebo output:** In the tmux pane where `gz sim` runs (window 0, left), look for “could not find model”, “failed to load”, “resource path”, “plugin … failed” to see which asset is missing.
- **Result:** Gazebo loads the world and shows entities (boat, water, buoys, etc.); then SITL can connect to the ArduPilotPlugin and you can arm and use GUIDED.

### 11. “DO SET MODE failed: ap: flight mode change failed” (GUIDED / arm)

- **Symptom:** In MAVProxy you run `mode GUIDED` and/or `arm throttle` and get **DO SET MODE failed: ap: flight mode change failed**.
- **Cause:** ArduPilot is refusing the mode change. This usually happens when it has no position estimate or fails pre-arm checks. If Gazebo shows **0 entities**, the boat (and ArduPilotPlugin) never loaded, so SITL never receives pose/IMU from Gazebo on port 9002 — ArduPilot then has no “vehicle state” and will reject GUIDED and arming.
- **Fix:**
  1. **Fix “0 entities” first** (see §10). Once the world and ourboat load, the plugin feeds SITL; then mode change and arming can succeed.
  2. **Relax arming for sim:** In MAVProxy run:
     ```text
     param set ARMING_REQUIRE 0
     mode GUIDED
     arm throttle
     ```
  3. If you already have entities and still get the error, check the SITL/MAVProxy pane for “need position estimate” or gyro/accel errors; ensure SITL is actually connected to the plugin (you should see activity on 9002 when the boat is in the world).
- **Result:** After the world loads and you set `ARMING_REQUIRE 0`, GUIDED and `arm throttle` succeed.

### 12. Gazebo “0 entities” again + libEGL / Mesa / nvidia-drm warnings (Jetson)

- **Symptom:** Gazebo shows **0 entities** again; in the Gazebo (or terminal) output you see **libEGL** warnings, **MESA-LOADER**, or **nvidia-drm** messages. The world or GUI may fail to load properly.
- **Cause:** The display/rendering stack is using Mesa’s software or a wrong EGL/OpenGL path instead of the NVIDIA driver, so Gazebo (or its GUI) can’t initialise the scene correctly. That can happen after a driver update, a bad login session, or if `LD_LIBRARY_PATH` doesn’t put NVIDIA libs first.
- **Fix:**
  1. **Force NVIDIA DRM mode** (one-time or after boot):  
     `sudo modprobe nvidia-drm modeset=1`
  2. **Run Gazebo with NVIDIA libs first:** The script already sets  
     `LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:...`  
     If you start `gz sim` by hand, set that too. Ensure no other `LD_LIBRARY_PATH` or `LIBGL_ALWAYS_SOFTWARE=1` is set in the same shell.
  3. **Force NVIDIA EGL/GL vendor** (if Mesa still wins):  
     Before starting the sim:  
     `export __GLX_VENDOR_LIBRARY_NAME=nvidia`  
     and optionally  
     `export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`  
     (path may differ; check `ls /usr/share/glvnd/egl_vendor.d/`).
  4. **Re-check “0 entities” path cause:** Even with a good display, 0 entities can still be due to `GZ_SIM_RESOURCE_PATH` or missing world/models (see §10). In the tmux pane where `gz sim` runs, look for “could not find model”, “resource path”, or EGL/load errors.
- **Result:** Gazebo uses the NVIDIA driver, the window and world load, and entities appear.

---

## One-Time Setup and Dependencies

- **ROS 2 Humble** and **Gazebo Harmonic** (gz-sim 8.x).
- **MAVProxy:** `pip install --user MAVProxy`; ensure `~/.local/bin` is on `PATH` (script sets it for the SITL window).
- **ArduPilot:** Clone or symlink to `~/ardupilot`; build SITL (e.g. `./waf configure --board sitl && ./waf build`); run from `~/ardupilot/Tools/autotest` with `sim_vehicle.py`.
- **ardupilot_gazebo:** Clone to `~/ardupilot_gazebo`, build as above; `libArduPilotPlugin.so` in `~/ardupilot_gazebo/build/`.
- **SITL_Models:** Boat model under `~/SITL_Models/Gazebo/models/ourboat/` (and any other models/worlds you use).
- **bridge_ws:** ROS 2 workspace with `ros_gz_bridge` (e.g. `~/bridge_ws`); script sources `~/bridge_ws/install/setup.bash`.
- **asv_wave_sim:** Built so that `~/autonomy-ws-25-26/src/asv_wave_sim/install/` exists and contains the waves/hydro libs and setup.
- **Git:** If ArduPilot is in another user’s directory, run `git config --global --add safe.directory <path>` once.

---

## Troubleshooting

| Symptom | What to check |
|--------|----------------|
| Boat falls through water | `LD_LIBRARY_PATH` includes `.../asv_wave_sim/install/lib`; Hydrodynamics `<enable>` is `ourboat::base_link`. |
| Boat doesn’t move in Gazebo | `GZ_SIM_SYSTEM_PLUGIN_PATH` includes `~/ardupilot_gazebo/build`; plugin rebuilt for current user; SITL starts after Gazebo (script delay). In Gazebo pane look for “ArduPilot”, “9002”, or plugin load errors. |
| MAVProxy not found | Script exports `PATH` and `PYTHONPATH` for the SITL window; `~/.local/bin/mavproxy.py` exists; use `python3 ./sim_vehicle.py`. |
| MAVROS connection refused | SITL listening on 5760; use `fcu_url:=tcp://localhost:5760`; wait for SITL to be up (script waits for port 5760). |
| Can’t arm: position / gyros | For sim, `param set ARMING_REQUIRE 0` then `mode GUIDED` and `arm throttle`. |
| Gazebo black or white screen | `LD_LIBRARY_PATH` for NVIDIA libs (Jetson); try `--render-engine-gui ogre` on `gz sim`; ensure NVIDIA drivers installed. |
| SITL build: Permission denied .wafpickle | Run `fix_ardupilot_build_permissions.sh` on the machine with `~/ardupilot`, or `chown -R $USER:$USER ~/ardupilot`, `rm -rf ~/ardupilot/build/sitl`, then `./waf configure` and `./waf rover` (no sudo). |
| Gazebo shows 0 entities | `GZ_SIM_RESOURCE_PATH` must include `.../gz-waves-models/models` and `.../gz-waves-models/world_models` (for waves, buoys, gates) plus `~/SITL_Models/...` (ourboat). Script has this; ensure those dirs exist. Check Gazebo tmux pane for “could not find model” errors. |
| Gazebo 0 entities + libEGL / Mesa / nvidia-drm | Run `sudo modprobe nvidia-drm modeset=1`. Ensure `LD_LIBRARY_PATH` starts with `/usr/lib/aarch64-linux-gnu/nvidia`; try `export __GLX_VENDOR_LIBRARY_NAME=nvidia`. See §12. |
| DO SET MODE failed: flight mode change failed / stuck on MANUAL | **Set the param first:** `param set ARMING_REQUIRE 0`, then `mode GUIDED`, then `arm throttle`. If you run `mode GUIDED` before setting the param, the mode change is rejected. |
| SITL crash / Git errors | Add ArduPilot repo path as `safe.directory`; ensure you run from `~/ardupilot/Tools/autotest`. |

**Restart everything:** Run `simulation_full.bash` again; it kills the existing `simfull` session and starts clean.

---

## Summary

- **One script:** `simulations/simulation_full.bash` starts Gazebo, bridges, MAVROS, and SITL in a tmux session.
- **One world:** `aquatonomous_world.sdf` in `simulations/worlds/`.
- **One boat model:** `ourboat` in `~/SITL_Models/Gazebo/models/ourboat/` with ArduPilotPlugin and Hydrodynamics enabled for `ourboat::base_link`.
- **Ports:** 5760 (MAVROS/SITL), 9002 (SITL ↔ Gazebo plugin).
- **Users:** Use `~/ardupilot` (symlink if needed), build plugin under your user, set Git safe directory if repo is under another user.

With this, the boat floats, arms, and moves in Gazebo under MAVProxy and ROS 2.
