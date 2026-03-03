# Manual Launch Guide – Running Simulation Components in Separate Terminals

This guide shows how to run each part of the simulation stack in its own terminal window, without tmux. This makes it easier to see errors and debug issues.

---

## Prerequisites

Before running any of these, ensure you have:

1. **ROS 2 Jazzy installed:** `/opt/ros/jazzy/setup.bash` must exist
2. **asv_wave_sim built:** Run the build command below if not done yet
3. **External dependencies (optional):** See [EXTERNAL_SETUP.md](EXTERNAL_SETUP.md)

---

## Quick Start: Test Gazebo Only

**Terminal 1 – Gazebo with course models (no boat, no MAVROS/SITL):**

```bash
cd ~/autonomy-ws-25-26
bash src/asv_wave_sim/test_gazebo_standalone.sh
```

This runs Gazebo with:
- Water (waves)
- Buoys, gates, rope holds
- **No boat** (unless you have `~/SITL_Models` set up)

Expected: Gazebo opens with water and course models. If you see "Unable to find uri[model://ourboat]", that's expected without `~/SITL_Models`.

---

## Full Stack: Gazebo + Bridges + MAVROS + SITL

For the complete simulation with the boat, MAVROS, and ArduPilot SITL, run these commands in separate terminals:

### Terminal 1 – Gazebo

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

**Wait for Gazebo to fully load and show entities before starting SITL (next step).**

---

### Terminal 2 – ros_gz_bridge (Lidar)

Wait ~10 seconds after Gazebo starts, then:

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /lidar_topic/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  --ros-args -r /lidar_topic/points:=/laser_points
```

---

### Terminal 3 – ros_gz_bridge (Camera 1)

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera1_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera1_topic:=/camera0/image_raw
```

---

### Terminal 4 – ros_gz_bridge (Camera 2)

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera2_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera2_topic:=/camera1/image_raw
```

---

### Terminal 5 – ros_gz_bridge (Camera 3)

```bash
source /opt/ros/jazzy/setup.bash
source ~/bridge_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /camera3_topic@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /camera3_topic:=/camera2/image_raw
```

---

### Terminal 6 – MAVROS

Wait ~35 seconds after Gazebo starts (so SITL has time to bind port 5760), then:

```bash
source /opt/ros/jazzy/setup.bash

# Wait for SITL port 5760 to be ready
until (echo >/dev/tcp/localhost/5760) 2>/dev/null; do 
  echo "Waiting for SITL on port 5760..."; 
  sleep 2; 
done

ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760
```

---

### Terminal 7 – ArduPilot SITL + MAVProxy

**Important:** Start this ~40 seconds after Gazebo starts, so the ArduPilotPlugin is loaded and listening on port 9002.

```bash
export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"

cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON \
  --console --map \
  --custom-location="51.566151,-4.034345,10.0,-135"
```

Once MAVProxy starts, you can control the boat:

```text
param set ARMING_REQUIRE 0
mode GUIDED
arm throttle
position 10 0 0
```

---

## Computer Vision (Optional)

### Terminal 8 – CV Pipeline (sim mode)

After the simulation is running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/autonomy-ws-25-26/computer_vision/install/setup.bash

ros2 launch cv_ros_nodes launch_cv_sim.py single_camera:=true
```

---

## Build Commands (First Time Setup)

### Build asv_wave_sim

```bash
cd ~/autonomy-ws-25-26/src/asv_wave_sim

# Install dependencies
sudo apt install -y libcgal-dev libfftw3-dev

# Build
colcon build --symlink-install --merge-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=ON \
  -DCMAKE_CXX_STANDARD=17

# Source it
source install/setup.bash
```

### Build bridge_ws (ros_gz_bridge for Jazzy)

```bash
mkdir -p ~/bridge_ws/src
cd ~/bridge_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b ros2

cd ~/bridge_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Gazebo shows "Unable to find uri[model://ourboat]" | Expected if `~/SITL_Models` not set up. See [EXTERNAL_SETUP.md](EXTERNAL_SETUP.md) |
| Gazebo shows "Unable to find uri[model://waves]" | Build `asv_wave_sim` and export `GZ_SIM_RESOURCE_PATH` |
| MAVROS connection refused | Wait for SITL to start and bind port 5760 |
| Boat doesn't move in Gazebo | Check that ArduPilotPlugin loaded (look for "ArduPilot" in Gazebo output). Ensure SITL started after Gazebo (40s delay). |
| "need position estimate" / can't arm | In MAVProxy: `param set ARMING_REQUIRE 0` then `mode GUIDED` then `arm throttle` |

---

## Summary

**Minimal test (Gazebo only):**
```bash
bash ~/autonomy-ws-25-26/src/asv_wave_sim/test_gazebo_standalone.sh
```

**Full stack:**
1. Terminal 1: Gazebo
2. Terminals 2-5: ros_gz_bridge (wait 10s after Gazebo)
3. Terminal 6: MAVROS (wait 35s after Gazebo)
4. Terminal 7: SITL (wait 40s after Gazebo)
5. Terminal 8 (optional): CV pipeline

For a single-command tmux-based launch, use:
```bash
bash ~/autonomy-ws-25-26/src/asv_wave_sim/aquatonomous_simulation_full.bash
```
