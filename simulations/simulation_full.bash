#!/bin/bash
# Full sim: win 0 = Gazebo+bridges, 1 = MAVROS+shell, 2 = ArduPilot. MAVROS connects to SITL on 5760.
# Run from anywhere. ArduPilot must be at ~/ardupilot.
# Switch windows: Ctrl+b 0 / 1 / 2
# See simulations/README.md for full documentation.

SESSION=simfull

# Start fresh: kill old session so Gazebo actually starts
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Window 0: Gazebo (from world dir) + 4 bridge panes (lidar, cam1, cam2, cam3)
# Layout: left = Gazebo; right = 4 vertical panes for bridges
# Use NVIDIA GPU libs first so ogre2 can render. Ensure ArduPilot plugin and ourboat model load so SITL (port 9002) can drive the boat in Gazebo.
# On Jetson, if Gazebo shows black/white screen, append: --render-engine-gui ogre
# If you see libEGL/Mesa/nvidia-drm and 0 entities, run before this script: sudo modprobe nvidia-drm modeset=1; export __GLX_VENDOR_LIBRARY_NAME=nvidia
tmux new-session -d -s "$SESSION" \
  'source /opt/ros/humble/setup.bash && source ~/autonomy-ws-25-26/src/asv_wave_sim/install/setup.bash && export GZ_VERSION=harmonic && export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:$HOME/autonomy-ws-25-26/src/asv_wave_sim/install/lib:$LD_LIBRARY_PATH && export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-} && export GZ_SIM_RESOURCE_PATH=$HOME/autonomy-ws-25-26/simulations/models:$HOME/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/models:$HOME/autonomy-ws-25-26/src/asv_wave_sim/gz-waves-models/world_models:$HOME/SITL_Models/Gazebo/models:$HOME/SITL_Models/Gazebo/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-} && cd ~/autonomy-ws-25-26/simulations/worlds && gz sim -v 4 -r aquatonomous_world.sdf'

tmux split-window -t "${SESSION}:0" -h 'source /opt/ros/humble/setup.bash && source ~/bridge_ws/install/setup.bash && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /lidar_topic/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked --ros-args -r /lidar_topic/points:=/laser_points'
tmux split-window -t "${SESSION}:0.1" 'source /opt/ros/humble/setup.bash && source ~/bridge_ws/install/setup.bash && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera1_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera1_topic:=/camera0/image_raw'
tmux split-window -t "${SESSION}:0.2" 'source /opt/ros/humble/setup.bash && source ~/bridge_ws/install/setup.bash && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera2_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera2_topic:=/camera1/image_raw'
tmux split-window -t "${SESSION}:0.3" 'source /opt/ros/humble/setup.bash && source ~/bridge_ws/install/setup.bash && sleep 20 && ros2 run ros_gz_bridge parameter_bridge /camera3_topic@sensor_msgs/msg/Image@gz.msgs.Image --ros-args -r /camera3_topic:=/camera2/image_raw'

# Window 1: MAVROS + shell (with prompt). Wait for SITL port 5760 so we don't get connection refused.
tmux new-window -t "$SESSION" -n mavros 'source /opt/ros/humble/setup.bash && sleep 35 && until (echo >/dev/tcp/localhost/5760) 2>/dev/null; do echo "Waiting for SITL on 5760..."; sleep 2; done && ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760'
tmux split-window -t "${SESSION}:1" -h 'source /opt/ros/humble/setup.bash && exec bash'

# Window 2: ArduPilot SITL (left; type mode GUIDED / arm throttle in MAVProxy) + shell (right; prompt)
# SITL starts after 40s so Gazebo and ArduPilotPlugin (port 9002) are ready. PATH must include ~/.local/bin for mavproxy.py.
# Left pane stays open on crash so you can read the error.
tmux new-window -t "$SESSION" -n ardupilot 'sleep 40 && ( export PATH="$HOME/.local/bin:$PATH"; export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"; cd ~/ardupilot/Tools/autotest && python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON --console --map --custom-location="51.566151,-4.034345,10.0,-135" ) || { echo "--- SITL exited. Restart with: export PATH=\"\$HOME/.local/bin:\$PATH\"; export PYTHONPATH=\"\$HOME/.local/lib/python3.10/site-packages:\$PYTHONPATH\"; cd ~/ardupilot/Tools/autotest && python3 ./sim_vehicle.py -v Rover -f rover-skid --model JSON --console --map --custom-location=\"51.566151,-4.034345,10.0,-135\""; exec bash; }'
tmux split-window -t "${SESSION}:2" -h 'source /opt/ros/humble/setup.bash && echo "Shell: use ros2 topics here, or restart SITL if the other pane crashed." && exec bash'

# Attach and show Gazebo (window 0) so you see the sim running
tmux select-window -t "${SESSION}:0"
tmux attach-session -t "$SESSION"
