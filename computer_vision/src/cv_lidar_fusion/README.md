# CV-LiDAR Fusion (inside computer_vision)

Moved from `autonomy-ws-25-26/cv_lidar_fusion` into the computer_vision directory.

**Subscribes to:** `/combined/detection_info` (CV), `/tracked_buoys` (LiDAR)  
**Publishes to:** `/fused_buoys` (TrackedBuoyArray with color from CV)

Requires `pointcloud_filters` from the mapping workspace. Build from a workspace that contains both mapping and computer_vision, or add mapping as a dependency path.

```bash
# From workspace that has both mapping and computer_vision
colcon build --packages-select pointcloud_filters cv_lidar_fusion
source install/setup.bash
ros2 run cv_lidar_fusion vision_lidar_fusion
```

See `computer_vision/FUSION_NODE_GUIDE.md` for full pipeline and message formats.
