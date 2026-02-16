# Computer Vision – Competition and testing

What to run at competition, when to use which launch, and how to iterate (calibration, overrides, tuning). For pipeline and node details see [NODES.md](NODES.md). For full design and hardware see [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md).

---

## Quick start (one command)

```bash
cd ~/autonomy-ws-25-26/computer_vision
source /opt/ros/humble/setup.bash
colcon build --packages-select cv_ros_nodes
source install/setup.bash
./set_camera_fps.sh
ros2 launch cv_ros_nodes launch_cv.py
```

Set 15 fps on cameras before launch (`./set_camera_fps.sh`; uses Think USB paths; on other hardware see [DESIGN_STRATEGY.md](DESIGN_STRATEGY.md) or override `camera_devices`).

---

## Task-specific launch (RoboBoat 2026)

| Task | Launch command |
|------|----------------|
| **Tasks 1–3** (Evacuation Route, Debris Clearance, Emergency Sprint) | `ros2 launch cv_ros_nodes launch_cv.py enable_indicator_buoy:=true enable_task4:=false enable_number_detection:=false` |
| **Task 4** (Supply Drop) | `ros2 launch cv_ros_nodes launch_cv.py enable_task4:=true enable_indicator_buoy:=false enable_number_detection:=false` |
| **Task 5** (Navigate the Marina) | `ros2 launch cv_ros_nodes launch_cv.py enable_number_detection:=true enable_indicator_buoy:=false enable_task4:=false` |
| **Task 6** (Harbor Alert) | Use base pipeline or same as Task 3/5; CV supports navigation (yellow buoy, dock). |

For Task 5, ensure the number detection TensorRT engine exists (e.g. build from `task_specific/Docking/number_detection/` with `build_number_engine.sh`). Override if needed: `number_detection_engine:=/path/to/number_detection.engine`.

---

## Competition settings: detections and accuracy

The robot uses **`/combined/detection_info`** (and `..._with_distance`) when inference runs and publishes. More inference runs = more detection updates.

| Goal | Setting | Why |
|------|--------|-----|
| **Most detections, best accuracy** | **`inference_interval:=1`** (default) | Run inference on every frame. Max detections/sec and lowest latency. |
| **Faster inference** | **FP16 TensorRT engine** | Build with `--fp16` (~2× faster). See [model_building_and_training/TENSORRT.md](model_building_and_training/TENSORRT.md). |
| **Confidence** | **`conf_threshold:=0.25`** (default) | Use **`0.2`** only if you need more marginal detections and accept more false positives. |
| **Real cameras at competition** | **`ros2 launch cv_ros_nodes launch_cv.py`** (+ task flags) | Default inference_interval is 1. Do not use sim launch for real cameras. |
| **Sim / testing** | **`launch_cv_sim.py`** with **`inference_interval:=1`** | Matches competition. Use `inference_interval:=2` only for smoother display when debugging. |
| **Jetson GPU memory** | **`single_camera:=true`** (sim only) | If OOM with sim + CV, run one camera. On real hardware use as many cameras as the GPU can handle. |

**Summary:** Use **`launch_cv.py`** (real cameras) with task flags above. Build **`model.engine`** with **FP16**. Keep **`inference_interval=1`** (default).

**Build FP16 engine (before competition):**
```bash
cd ~/autonomy-ws-25-26/computer_vision
python model_building_and_training/export_onnx.py model_building_and_training/aqua_main.pt
/usr/src/tensorrt/bin/trtexec --onnx=model_building_and_training/aqua_main.onnx --saveEngine=cv_scripts/model.engine --fp16 --memPoolSize=workspace:4096 --skipInference
```

---

## Distance calibration (one-point)

The pipeline multiplies distance estimates by a **distance scale factor**. Use one or more photos at known distance/size to compute it; pass at launch (e.g. `distance_scale_factor:=0.87`). See **[camera_calibration/README.md](camera_calibration/README.md)** for steps and CSV-based calibration. Recompute after adding calibration photos.

---

## Iterating at competition

- **Override engine path:** `engine_path:=/path/to/model.engine`
- **Override resolution:** `resolution:=1920,1200`
- **Override cameras:** `camera_devices:=/path1,/path2,/path3` (exactly 3)
- **Simulation:** Use **`launch_cv_sim.py`** or **`launch_cv.py use_sim:=true`** when Gazebo bridges publish `/camera0/image_raw` etc. See [../simulations/README.md](../simulations/README.md).

For manual node commands and topic I/O, see [NODES.md](NODES.md).
