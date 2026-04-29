# Web Server Map – 2D Map Visualization

Lightweight web-based 2D map for boat position and global detections. Runs on the Jetson, viewed from your laptop via SSH port forwarding.

## 1. One-time setup (Jetson)

```bash
pip3 install aiohttp
cd ~/autonomy-ws-25-26
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
```

## 2. Live map from running system

**On the Jetson (map node):**

```bash
cd ~/autonomy-ws-25-26
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run web_server_map map_visualizer_node
```

This node subscribes to:

- `/boat_pose` (`global_frame/BoatPose`)
- `/global_detections` (`global_frame/GlobalDetectionArray`)

**From your laptop (tunnel + browser):**

```bash
ssh -L 8080:localhost:8080 user@jetson-ip
```

Then open `http://localhost:8080` in a browser.

## 3. Replay a recorded `map_data_*` bag

**On the Jetson:**

Terminal 1 – start the visualizer:

```bash
cd ~/autonomy-ws-25-26
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run web_server_map map_visualizer_node
```

Terminal 2 – from the repo root, replay a bag:

```bash
cd ~/autonomy-ws-25-26
./replay_map.sh map_data_YYYYMMDD_HHMMSS   # or omit arg for latest map_data_*
```

Then from your laptop:

```bash
ssh -L 8080:localhost:8080 user@jetson-ip
```

Open `http://localhost:8080` in a browser to see the map.

## 4. Launch file (optional)

```bash
cd ~/autonomy-ws-25-26
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch web_server_map map_visualizer.launch.py
# or, for custom settings:
ros2 launch web_server_map map_visualizer.launch.py port:=8888 update_rate_hz:=5.0
```

## 5. Troubleshooting (quick)

- **Node won’t start**: `pip3 show aiohttp`; ensure you built from root and sourced `install/setup.bash`.
- **Map says “waiting for data”**: check `ros2 topic list` and `ros2 topic echo /boat_pose` on the Jetson.
- **Can’t reach from laptop**: verify tunnel (`ssh -L 8080:localhost:8080 user@jetson-ip`) and open `http://localhost:8080`.


##
```bash
source /opt/ros/humble/setup.bash
./build.sh
source install/setup.bash
pkill -9 -f "map_visualizer_node" 2>/dev/null
sleep 1
ros2 run web_server_map map_visualizer_node
```