# Computer Vision ROS2 Pipeline

Multi-camera object detection with TensorRT-optimized YOLO on ROS2 Humble: 3 cameras → preprocessing → inference → combiner. For architecture and design, see [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md).

---

## Quick start

```bash
cd ~/autonomy-ws-25-26/computer_vision
source /opt/ros/humble/setup.bash
colcon build --packages-select cv_ros_nodes
source install/setup.bash
```

Set 15 fps on cameras before launch (see [NODES_AND_CAMERAS.md](NODES_AND_CAMERAS.md)):

```bash
for d in /dev/video0 /dev/video2 /dev/video4; do v4l2-ctl -d "$d" --set-parm 15; done
```

```bash
ros2 launch cv_ros_nodes launch_cv.py
# With Task4 supply processor: enable_task4:=true
```

For launch overrides (`enable_task4:=true`, `resolution`, `camera_devices`, `engine_path`, etc.), manual node commands, and monitoring: [NODES_AND_CAMERAS.md](NODES_AND_CAMERAS.md). Downstream (e.g. mission/control): use `/combined/detection_info`; detections are in the global frame 1920×480; `bbox` is `[x1,y1,x2,y2]` in that frame; `global_frame: { "width": 1920, "height": 480 }`. Task4 supply drops have `source == "task4"` and `type` in `"yellow_supply_drop"`, `"black_supply_drop"`.

---

## Documentation

| Doc | Description |
|-----|-------------|
| [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md) | Design and architecture |
| [NODES_AND_CAMERAS.md](NODES_AND_CAMERAS.md) | Nodes, cameras, launch, monitoring, topics, detection_info |
| [SIMULATIONS.md](SIMULATIONS.md) | Running the sim |
| [TENSORRT.md](TENSORRT.md) | TensorRT setup and model conversion |
| [FUSION_NODE_GUIDE.md](FUSION_NODE_GUIDE.md) | CV + LiDAR fusion |

---

## Prerequisites

- **OS**: Ubuntu 22.04  
- **ROS2**: Humble  
- **Python**: 3.10+  
- **GPU**: NVIDIA with CUDA  

**ROS2 packages:** `ros-humble-v4l2-camera`, `ros-humble-cv-bridge`  

**Python:** `numpy<2`, `opencv-python`, `tensorrt`, `pycuda`

---

## Project structure

```
computer_vision/
├── src/cv_ros_nodes/          # cv_ros_nodes package (nodes, launch_cv.py)
├── cv_scripts/                 # model.engine, class_mapping.yaml
├── model_training/             # weights.pt, weights.onnx, test_inference.py
├── README.md
├── DESIGN_STRATEGY.md
├── NODES_AND_CAMERAS.md
├── SIMULATIONS.md
├── TENSORRT.md
└── FUSION_NODE_GUIDE.md
```

---

## License

MIT License. Contributors: Lorenzo DeMarni.

*Last updated: January 2025*
