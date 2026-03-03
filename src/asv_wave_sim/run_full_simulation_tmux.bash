#!/bin/bash
# Run full boat simulation in tmux: Gazebo + bridges + MAVROS + ArduPilot SITL
# Window 0: Gazebo + 4 bridge panes (lidar, camera1, camera2, camera3)
# Window 1: MAVROS + shell
# Window 2: ArduPilot SITL + MAVProxy
# Switch windows: Ctrl+b 0 / 1 / 2
# See SIMULATION.md for full documentation.

set -e

SESSION=simfull

# Compute absolute paths so we don't depend on $HOME being correct inside tmux.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/../.." >/dev/null 2>&1 && pwd)"
USER_HOME="${HOME}"
ASV_INSTALL="${WS_ROOT}/install"

# Start fresh: kill old session so Gazebo actually starts
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Helper: send a command string to a pane and press Enter
send() { tmux send-keys -t "$1" "$2" Enter; }

# ── Create session and all windows/panes with plain bash ─────────────────────
tmux new-session  -d -s "$SESSION" -x 220 -y 50
# Window 0: already exists (pane 0.0 = Gazebo). Add 4 bridge panes on the right.
tmux split-window -t "${SESSION}:0" -h
tmux split-window -t "${SESSION}:0.1" -v
tmux split-window -t "${SESSION}:0.2" -v
tmux split-window -t "${SESSION}:0.3" -v

# Window 1: SITL (left) + ROS shell (right)
tmux new-window -t "${SESSION}" -n sitl
tmux split-window -t "${SESSION}:1" -h

# Window 2: MAVROS (left) + ROS shell (right)
tmux new-window -t "${SESSION}" -n mavros
tmux split-window -t "${SESSION}:2" -h

# ── Window 0: Gazebo + Bridges ───────────────────────────────────────────────
send "${SESSION}:0.0" "source /opt/ros/jazzy/setup.bash && source ${ASV_INSTALL}/setup.bash && export GZ_VERSION=harmonic && export GZ_CONFIG_PATH=/usr/share/gz:\${GZ_CONFIG_PATH:-} && export LD_LIBRARY_PATH=${ASV_INSTALL}/lib:\$LD_LIBRARY_PATH && export GZ_SIM_SYSTEM_PLUGIN_PATH=${ASV_INSTALL}/lib:${USER_HOME}/ardupilot_gazebo/build && export GZ_SIM_RESOURCE_PATH=${WS_ROOT}/src/asv_wave_sim/gz-waves-models/models:${WS_ROOT}/src/asv_wave_sim/gz-waves-models/world_models:${USER_HOME}/SITL_Models/Gazebo/models:${USER_HOME}/SITL_Models/Gazebo/worlds:${USER_HOME}/ardupilot_gazebo/models:${USER_HOME}/ardupilot_gazebo/worlds && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && cd ${WS_ROOT}/src/asv_wave_sim/gz-waves-models/worlds && gz sim -v 4 -r aquatonomous_world.sdf"

# Bridge panes – sleep 20s to let Gazebo fully initialize
BRIDGE_SRC="source /opt/ros/jazzy/setup.bash; [ -f ~/bridge_deps_ws/install/setup.bash ] && source ~/bridge_deps_ws/install/setup.bash; [ -f ~/bridge_ws/install/setup.bash ] && source ~/bridge_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
send "${SESSION}:0.1" "$BRIDGE_SRC && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /lidar_topic/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked --ros-args -r /lidar_topic/points:=/laser_points"
send "${SESSION}:0.2" "$BRIDGE_SRC && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera1_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera1_topic:=/camera0/image_raw"
send "${SESSION}:0.3" "$BRIDGE_SRC && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera2_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera2_topic:=/camera1/image_raw"
send "${SESSION}:0.4" "$BRIDGE_SRC && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera3_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera3_topic:=/camera2/image_raw"

# ── Window 1: ArduPilot SITL ─────────────────────────────────────────────────
# MAVProxy runs inside this pane. --console and --map open Qt sub-windows (OK).
# Waits 15s for Gazebo/ArduPilotPlugin to be ready on port 9002.
send "${SESSION}:1.0" "export PATH=${USER_HOME}/.local/bin:\$PATH && export PYTHONPATH=${USER_HOME}/.local/lib/python3.12/site-packages:\$PYTHONPATH && sleep 15 && echo 'Starting SITL...' && cd ~/ardupilot/Tools/autotest && python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON --console --map --custom-location=51.566151,-4.034345,10.0,-135"
send "${SESSION}:1.1" "source /opt/ros/jazzy/setup.bash && source ${ASV_INSTALL}/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && echo 'ROS shell - try: ros2 topic list'"

# ── Window 2: MAVROS ─────────────────────────────────────────────────────────
# Waits for SITL TCP port 5760 before launching MAVROS.
send "${SESSION}:2.0" "source /opt/ros/jazzy/setup.bash && source ${ASV_INSTALL}/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && echo 'Waiting for SITL port 5760...' && until (echo >/dev/tcp/localhost/5760) 2>/dev/null; do echo '  waiting...'; sleep 3; done && echo 'SITL ready! Starting MAVROS...' && ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760"
send "${SESSION}:2.1" "source /opt/ros/jazzy/setup.bash && source ${ASV_INSTALL}/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && echo 'ROS shell - try: ros2 topic list | grep mavros'"

# Attach at Window 0 (Gazebo view)
tmux select-window -t "${SESSION}:0"
tmux attach-session -t "$SESSION"
