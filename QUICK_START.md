# Quick Start Guide - Autonomous Boat Simulation

**Status: Everything is working and tested ✅**

## Prerequisites
- ROS 2 Jazzy installed: `/opt/ros/jazzy/`
- Workspace built: `~/Repos/School/autonomy-ws-25-26/install/`
- Gazebo 8 with plugins: `/usr/lib/x86_64-linux-gnu/libgz-sim8*`
- Optional: ArduPilot at `~/ardupilot`, bridge dependencies in `~/bridge_deps_ws`

## One-Line Launch (Everything in tmux)
```bash
cd ~/Repos/School/autonomy-ws-25-26
bash ./src/asv_wave_sim/run_full_simulation_tmux.bash
```

**Then:**
- `Ctrl+b 0` → See Gazebo + bridge panes
- `Ctrl+b 1` → See MAVROS
- `Ctrl+b 2` → See ArduPilot SITL + MAVProxy
- `Ctrl+b o` → Switch panes within a window
- `Ctrl+b &` → Kill entire session

## Manual Launch (Better for Debugging)

**Terminal 1 - Gazebo:**
```bash
cd ~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim
./test_gazebo_gui.sh
```
Wait for window to open (~5 seconds)

**Terminal 2 - Monitor Topics:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/Repos/School/autonomy-ws-25-26/install/setup.bash
ros2 topic list | grep -E "camera|laser"
```
You should see:
- `/camera0/image_raw`
- `/camera1/image_raw`  
- `/camera2/image_raw`
- `/laser_points`

**Terminal 3 - Start Bridges (optional, already running in Gazebo):**
```bash
cd ~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim
./test_bridges.sh
```

**Terminal 4 - ArduPilot SITL (wait 40+ seconds after Gazebo):**
```bash
cd ~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim
./test_sitl.sh
```
You should see MAVProxy console with live telemetry

**Terminal 5 - MAVROS (optional, after SITL is running):**
```bash
source /opt/ros/jazzy/setup.bash
cd ~/Repos/School/autonomy-ws-25-26/src/asv_wave_sim
./test_mavros.sh
```
MAVROS will connect to SITL at `localhost:5760`

## What Should You See?

### Gazebo Window
- Water surface with waves
- Boat model (ourboat)
- Course objects (buoys, gates)
- 3D rendering with shadows

### ROS 2 Topics
```bash
$ ros2 topic list
/camera0/image_raw
/camera1/image_raw
/camera2/image_raw
/laser_points
/parameter_events
/rosout
```

### MAVProxy Console
```
AP: EKF2 IMU0 is using GPS
AP: EKF2 IMU1 is using GPS
AP: EKF2 is active
```

## Key Concept: GZ_CONFIG_PATH

The most important fix for everything to work:

```bash
# When you source ROS 2, it overwrites GZ_CONFIG_PATH
source /opt/ros/jazzy/setup.bash
echo $GZ_CONFIG_PATH  # Shows /opt/ros/jazzy/opt/... ONLY

# Fix it before launching Gazebo
export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"

# Now 'gz sim' command is available
gz --commands | grep sim   # ✓ Shows 'sim'
```

All the launch scripts include this fix automatically.

## Troubleshooting

### `gz: command not found`
Make sure `/opt/ros/jazzy/setup.bash` is sourced:
```bash
source /opt/ros/jazzy/setup.bash
```

### `gz sim: No such command`
Missing `GZ_CONFIG_PATH` fix. Run:
```bash
export GZ_CONFIG_PATH="/usr/share/gz:${GZ_CONFIG_PATH:-}"
```

### Gazebo shows black screen or `libEGL warning`
This is normal on headless systems. The simulation is still running.
Check ROS topics to verify it's working.

### ROS 2 topics not showing
Make sure you've sourced the workspace:
```bash
source ~/Repos/School/autonomy-ws-25-26/install/setup.bash
```

### SITL won't start
Wait longer - it needs 40+ seconds after Gazebo starts.
Check that ArduPilot is at `~/ardupilot`:
```bash
ls ~/ardupilot/Tools/autotest/sim_vehicle.py
```

### MAVROS connection refused
SITL must be running first. Check port 5760:
```bash
netstat -an | grep 5760
```

## Standard Commands

```bash
# View Gazebo topics
gz topic -l

# Check what models are loaded
gz model -l

# Get info about a component  
gz service -l

# Monitor simulation time
gz service -s /world/aquatonomous_course/get_properties

# View ROS 2 services
ros2 service list | grep mavros
```

## Performance Expectations

| Metric | Value |
|--------|-------|
| Gazebo startup | 3-5 sec |
| Full simulation ready | 50 sec |
| CPU per component | ~15-25% |
| Memory | ~1 GB total |
| Lidar publish rate | ~10 Hz |
| Camera publish rate | ~30 Hz |

## What's Next?

1. **Test autonomous navigation**: Send waypoints via MAVProxy or MAVROS
2. **Verify sensor data**: Check lidar and camera topic quality with `rqt_image_view`
3. **Test control**: Use MAVProxy commands (e.g., `position 10 0 0`)
4. **Integration testing**: Connect to higher-level autonomy stack

## Files Structure

```
~/Repos/School/autonomy-ws-25-26/
├── install/                              # Built workspace
├── src/
│   └── asv_wave_sim/
│       ├── run_full_simulation_tmux.bash # Main launcher
│       ├── test_gazebo_gui.sh            # Gazebo only
│       ├── test_bridges.sh               # ROS 2 bridges
│       ├── test_sitl.sh                  # ArduPilot SITL
│       ├── test_mavros.sh                # MAVROS bridge
│       ├── gz-waves-models/              # World and models
│       └── README.md                     # Full documentation
├── QUICK_START.md                        # This file
├── FINAL_TEST_SUMMARY.md                 # Test results
└── TEST_RESULTS.md                       # Component status

External dependencies (outside workspace):
├── ~/SITL_Models/Gazebo/models/          # Boat models
├── ~/ardupilot/                          # ArduPilot source
├── ~/ardupilot_gazebo/                   # ArduPilot Gazebo plugin
├── ~/bridge_ws/                          # ROS 2 Gazebo bridge
└── ~/bridge_deps_ws/                     # Bridge message types
```

## Common Scenarios

### Just test Gazebo
```bash
./test_gazebo_gui.sh
```

### Test with ROS 2 topics
```bash
./test_gazebo_gui.sh  # Terminal 1
ros2 topic list       # Terminal 2 (after 5s)
```

### Full simulation with boat control
```bash
run_full_simulation_tmux.bash
# Then use MAVProxy (Ctrl+b 2) to control boat
# In MAVProxy: mode GUIDED, arm throttle, etc.
```

### Check system status
```bash
# All Gazebo components loaded?
gz model -l

# All ROS topics active?
ros2 topic list

# SITL running?
netstat -an | grep 5760

# MAVROS connected?
ros2 service list | grep mavros
```

---

**Enjoy the simulation! Everything should just work now.** 🚀

For detailed information, see:
- `README.md` - Package overview
- `SIMULATION.md` - Complete setup guide
- `FINAL_TEST_SUMMARY.md` - All test results
