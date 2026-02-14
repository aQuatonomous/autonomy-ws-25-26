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
7. [Tmux Layout and Navigation](#tmux-layout-and-navigation)
8. [How to Control the Boat](#how-to-control-the-boat)
9. [Ports and Communication](#ports-and-communication)
10. [Issues We Hit and How We Fixed Them](#issues-we-hit-and-how-we-fixed-them)
11. [One-Time Setup and Dependencies](#one-time-setup-and-dependencies)
12. [Troubleshooting](#troubleshooting)

---

## What This Is

- **Gazebo Sim (gz sim)** runs a boat world (`super_cool_test.sdf`) with water, buoys, gates, and the **ourboat** model.
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

- **Gazebo** loads the world `super_cool_test.sdf` with water (waves model), buoys, gates, and the ourboat model.
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
| **World (map)** | `autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/worlds/super_cool_test.sdf` |
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

2. **From anywhere (if you copy the script):**
   ```bash
  cp ~/autonomy-ws-25-26/simulations/simulation_full.bash ~/simulation_full.bash
   chmod +x ~/simulation_full.bash
   ~/simulation_full.bash
   ```

The script will:

- Kill any existing tmux session named `simfull`.
- Start a new session with three windows (Gazebo+bridges, MAVROS, ArduPilot).
- Attach you to the session (if you’re in a real terminal); you’ll land on the Gazebo window.

If you run the script from an environment that can’t attach (e.g. some IDEs), the session still starts in the background. Attach manually:

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

```text
mode GUIDED
arm throttle
position 10 0 0     # 10 m north (NED: North, East, Down)
velocity 2 0 0      # 2 m/s north
position 5 5 0      # 5 m north, 5 m east
```

- Use **`position X Y Z`** (meters in NED from home) or **`velocity X Y Z`** (m/s in NED).
- Do **not** use `guided lat lon` in MAVProxy for Rover—that syntax is for other vehicle types and can cause errors or crashes. Use `position` / `velocity` or send global setpoints via ROS 2/MAVROS.

**Arming without GPS (simulation):**

If you see “need position estimate” or “gyros inconsistent” and want to arm anyway:

```text
param set ARMING_REQUIRE 0
mode GUIDED
arm throttle
```

**From ROS 2 (e.g. shell pane with ROS sourced):**

- Global position setpoint example:
  ```bash
  ros2 topic pub /mavros/setpoint_position/global geographic_msgs/msg/GeoPoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {latitude: 51.5662, longitude: -4.0343, altitude: 0.0}}}" --once
  ```
- Use MAVROS services for mode, arm, etc., as needed.

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

- **Symptom:** Possible load or link errors; Hydrodynamics not applying to the boat.
- **Cause:** Directory was `ourboat`, but the SDF had `<model name="blueboat">` and mesh URIs like `models://blueboat/...`.
- **Fix:** In `~/SITL_Models/Gazebo/models/ourboat/model.sdf`:
  - Set `<model name="ourboat">`.
  - Replace all `models://blueboat/` with `models://ourboat/`.
  - Set Hydrodynamics `<enable>ourboat::base_link</enable>` (as above).
- **Result:** Gazebo and plugins consistently use `ourboat`; no name/link mismatch.

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

### 8. Gazebo black screen (Jetson / headless)

- **Symptom:** Gazebo window opens but stays black.
- **Cause:** EGL/GPU drivers or library path; ogre2 not finding the right libs.
- **Fix:** Script already sets `LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:...`. If still black, add `--render-engine-gui ogre` to the `gz sim` command in `simulation_full.bash` (worse quality but more compatible).
- **Result:** Gazebo GUI becomes visible.

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
| Gazebo black screen | `LD_LIBRARY_PATH` for NVIDIA libs; try `--render-engine-gui ogre` on `gz sim`. |
| SITL crash / Git errors | Add ArduPilot repo path as `safe.directory`; ensure you run from `~/ardupilot/Tools/autotest`. |

**Restart everything:** Run `simulation_full.bash` again; it kills the existing `simfull` session and starts clean.

---

## Summary

- **One script:** `simulations/simulation_full.bash` starts Gazebo, bridges, MAVROS, and SITL in a tmux session.
- **One world:** `super_cool_test.sdf` in `src/asv_wave_sim/gz-waves-models/worlds/`.
- **One boat model:** `ourboat` in `~/SITL_Models/Gazebo/models/ourboat/` with ArduPilotPlugin and Hydrodynamics enabled for `ourboat::base_link`.
- **Ports:** 5760 (MAVROS/SITL), 9002 (SITL ↔ Gazebo plugin).
- **Users:** Use `~/ardupilot` (symlink if needed), build plugin under your user, set Git safe directory if repo is under another user.

With this, the boat floats, arms, and moves in Gazebo under MAVProxy and ROS 2.
