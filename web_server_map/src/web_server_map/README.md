# Web Server Map – 2D Map Visualization

Lightweight web-based 2D map for boat position and global detections. Real-time updates in a browser, no GUI on the Jetson; use SSH port forwarding for remote access.

**Independent workspace** – builds and runs separately from mapping, planning, and computer vision. Only listens to ROS2 topics at runtime.

## Features

- Real-time boat position with heading arrow
- Color-coded detection markers by class
- Auto-scaling coordinate system
- WebSocket updates (5–10 Hz)
- Grid overlay and mouse position
- Connection status indicator
- Low CPU and bandwidth use

## Quick Start

```bash
pip3 install aiohttp
cd ~/autonomy-ws-25-26/web_server_map
source /opt/ros/humble/setup.bash
source ../mapping/install/setup.bash  # for global_frame message types
colcon build --symlink-install
source install/setup.bash

ros2 run web_server_map map_visualizer_node
```

Open **http://localhost:8080**. For remote access: `ssh -L 8080:localhost:8080 user@jetson-ip`, then open **http://localhost:8080**.

## Installation

1. **Python dependency**

   ```bash
   pip3 install aiohttp
   ```

2. **Build**

   ```bash
   cd ~/autonomy-ws-25-26/web_server_map
   source /opt/ros/humble/setup.bash
   source ../mapping/install/setup.bash  # for global_frame message types
   colcon build --symlink-install
   source install/setup.bash
   ```

   Or use the main build script from root:
   ```bash
   cd ~/autonomy-ws-25-26
   ./build.sh  # builds all workspaces including web_server_map
   ```

## Usage

**Run node**

```bash
cd ~/autonomy-ws-25-26/web_server_map
source install/setup.bash
ros2 run web_server_map map_visualizer_node
```

**Launch file**

```bash
cd ~/autonomy-ws-25-26/web_server_map
source install/setup.bash
ros2 launch web_server_map map_visualizer.launch.py
ros2 launch web_server_map map_visualizer.launch.py port:=8888 update_rate_hz:=5.0
```

**Parameters**

- `port` (default: 8080) – HTTP server port
- `update_rate_hz` (default: 10.0) – WebSocket broadcast rate
- `boat_pose_topic` (default: `/boat_pose`)
- `global_detections_topic` (default: `/global_detections`)

**Access**

- Local: **http://localhost:8080**
- Remote: `ssh -L 8080:localhost:8080 user@jetson-ip`, then **http://localhost:8080**

## Topics

- **Subscribes**
  - `/boat_pose` (`global_frame/BoatPose`) – east, north, heading_rad
  - `/global_detections` (`global_frame/GlobalDetectionArray`) – east, north, class_name, source, id

- **Publishers**
  - None (read-only consumer)

## Architecture

- **ROS 2 node** (`map_visualizer_node.py`): subscribes to topics, keeps state, runs web server in a thread.
- **Web server** (aiohttp): serves static files, WebSocket at `/ws`, broadcasts state at configured rate.
- **Web client**: HTML5 canvas, WebSocket, auto-scaling, color-coded markers.

## Color Legend

- **Blue arrow** – Boat (heading)
- **Red / Green / Yellow / Black** – Red, green, yellow, black buoys
- **Dark red / Dark green** – Red/green pole buoys
- **Gray** – Unknown

## Coordinate System

- **Map**: ENU – East = +X, North = +Y; heading 0° = East, positive = CCW.
- **Canvas**: Auto-scaled to fit boat + detections with padding.

## Testing

**1. Run node and open http://localhost:8080**

**2. Publish test boat pose**

```bash
ros2 topic pub /boat_pose global_frame/BoatPose "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, east: 5.0, north: 3.0, heading_rad: 0.785}" --rate 1
```

**3. Publish test detections**

```bash
ros2 topic pub /global_detections global_frame/GlobalDetectionArray "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, detections: [{east: 10.0, north: 8.0, class_name: 'red_buoy', source: 'lidar', id: 1, range_m: 5.0, bearing_global_rad: 0.5}]}" --rate 1
```

You should see boat arrow at (5, 3) and red buoy at (10, 8). Mouse over canvas to see coordinates.

**4. Launch file**

```bash
cd ~/autonomy-ws-25-26/web_server_map
source install/setup.bash
ros2 launch web_server_map map_visualizer.launch.py
ros2 launch web_server_map map_visualizer.launch.py port:=8888
```

**5. SSH + live data**

On Jetson: run `boat_state_node` and `detection_to_global_node` (or full mapping stack), then run `map_visualizer_node`. From your machine: `ssh -L 8080:localhost:8080 user@jetson-ip` and open http://localhost:8080.

## Performance

- Update rate: 5–10 Hz (configurable)
- Bandwidth: ~1–5 KB per update
- CPU: low (JSON + network)
- Memory: low
- Latency: &lt;50 ms WebSocket + network

## Troubleshooting

| Issue | Fix |
|------|-----|
| Node won’t start | `pip3 show aiohttp`; ensure mapping workspace is built and sourced |
| Port in use | `ros2 run web_server_map map_visualizer_node --ros-args -p port:=8888` |
| "Waiting for data..." | Check `ros2 topic list` and `ros2 topic echo /boat_pose` |
| WebSocket drops | Reduce `update_rate_hz`, check network/firewall |
| Can’t reach via SSH | Verify tunnel: `ssh -L 8080:localhost:8080 user@jetson-ip`; try different local port |
| global_frame not found | Run `./build.sh` from root, or build mapping workspace first |

## File Structure

```
autonomy-ws-25-26/
├── web_server_map/              # Independent workspace
│   ├── src/web_server_map/      # ROS2 package
│   │   ├── src/web_server_map/
│   │   │   ├── __init__.py
│   │   │   ├── map_visualizer_node.py
│   │   │   └── web_server.py
│   │   ├── web/
│   │   │   ├── index.html
│   │   │   ├── map.js
│   │   │   └── style.css
│   │   ├── launch/
│   │   │   └── map_visualizer.launch.py
│   │   ├── resource/
│   │   │   └── web_server_map
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── CMakeLists.txt
│   │   └── README.md
│   ├── build/                   # Build artifacts
│   └── install/                 # Install space
├── mapping/                     # Mapping workspace (global_frame)
├── planning/                    # Planning workspace  
└── computer_vision/             # Computer vision workspace
```

## License

MIT
