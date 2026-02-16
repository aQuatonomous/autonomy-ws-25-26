# LiDAR pipeline – run now

**Build (from the mapping folder):**

```bash
cd ~/autonomy-ws-25-26/mapping
source /opt/ros/humble/setup.bash
colcon build --packages-select unitree_lidar_ros2 pointcloud_filters --symlink-install
source install/setup.bash
```

**Run the full pipeline (one command):**

```bash
cd ~/autonomy-ws-25-26/mapping
source install/setup.bash
ros2 launch pointcloud_filters buoy_pipeline.launch.py
```

RViz opens with point clouds and tracked buoy markers. Fixed frame: `unilidar_lidar`.

- **No LiDAR attached?** Use `launch_lidar_driver:=false`.
- **Overrides, inside-bay, tuning, troubleshooting:** [COMPETITION.md](COMPETITION.md).
- **WSL2 + USB LiDAR:** [WSL2_LIDAR_SETUP.md](WSL2_LIDAR_SETUP.md).
- **Full node and parameter details:** [README.md](README.md).
