# Computer Vision ROS2 Pipeline

Multi-camera object detection with TensorRT-optimized YOLO on ROS2 Humble: 3 cameras → preprocessing → inference → combiner. Final output: **`/combined/detection_info_with_distance`** (JSON, ~15 Hz) for navigation and CV–LiDAR fusion.

---

## Documentation

| Doc | Purpose |
|-----|--------|
| **[DESIGN_STRATEGY.md](DESIGN_STRATEGY.md)** | Full design: hardware, architecture, messages, training, calibration, task-specific requirements, failure modes, deployment. |
| **[COMPETITION.md](COMPETITION.md)** | What to run at competition and when; task-specific launch table; inference/FP16/calibration; iterating at competition. |
| **[NODES.md](NODES.md)** | Build, launch, manual node commands, topic inputs/outputs, final outputs. |

**See also:** [camera_calibration/README.md](camera_calibration/README.md) (distance scale factor), [model_building_and_training/TENSORRT.md](model_building_and_training/TENSORRT.md) (TensorRT and engine build), [FUSION_NODE_GUIDE.md](FUSION_NODE_GUIDE.md) (CV + LiDAR fusion), [DISTANCE_ESTIMATOR_CHANGES.md](DISTANCE_ESTIMATOR_CHANGES.md) (distance estimator), [simulations/README.md](../simulations/README.md) (sim + CV).

---

## Quick start

```bash
cd ~/autonomy-ws-25-26/computer_vision
source /opt/ros/humble/setup.bash
colcon build --packages-select cv_ros_nodes
source install/setup.bash
./set_camera_fps.sh
ros2 launch cv_ros_nodes launch_cv.py
```

Task-specific launch and competition settings: [COMPETITION.md](COMPETITION.md). Pipeline and nodes: [NODES.md](NODES.md).

---

## Project structure

```
computer_vision/
├── camera_calibration/     # Distance scale factor, calibration CSV
├── model_building_and_training/  # Weights, ONNX, TensorRT, TENSORRT.md
├── src/
│   ├── cv_ros_nodes/       # Nodes, launch_cv.py, launch_cv_sim.py
│   └── cv_lidar_fusion/    # vision_lidar_fusion → /fused_buoys
├── cv_scripts/             # model.engine, class_mapping.yaml
├── task_specific/          # Task 2/3, Docking number detection
├── set_camera_fps.sh
├── README.md
├── DESIGN_STRATEGY.md
├── COMPETITION.md
├── NODES.md
└── (FUSION_NODE_GUIDE.md, DISTANCE_ESTIMATOR_CHANGES.md)
```

---

## License

MIT License. Contributors: Lorenzo DeMarni.
