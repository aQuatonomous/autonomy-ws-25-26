# ROS2 Node Scripts

## Pipeline
```
Raw Camera → Preprocessing → Inference
/camera0/image_raw → /camera0/image_preprocessed
```

## Camera Nodes
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -p image_size:="[1920,1200]" -p framerate:=60.0 -r __ns:=/camera0
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video1 -p image_size:="[1920,1200]" -p framerate:=60.0 -r __ns:=/camera1
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video2 -p image_size:="[1920,1200]" -p framerate:=60.0 -r __ns:=/camera2
```

## CV Nodes
```bash
python3 vision_preprocessing.py  # Subscribes: /camera0/image_raw → Publishes: /camera0/image_preprocessed
python3 vision_inference.py       # Subscribes: /camera0/image_preprocessed
```

## Monitoring
```bash
ros2 topic hz /camera0/image_raw
ros2 topic hz /camera0/image_preprocessed
ros2 topic list
```
