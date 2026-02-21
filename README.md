# Roboboat 2026 - Autonomy Workspace

Welcome to the **Autonomy** repository for the Roboboat 2026 project.

## Current setup (this session)

- **Cameras:** 3× Arducam B0495 (USB3 2.3MP), on a powered USB hub (e.g. TP-Link UH700).  
  Positions are fixed by **by-path** in one place only: `computer_vision/set_camera_fps.sh`  
  - Cam0 = port **1.2**, Cam1 = **1.1**, Cam2 = **1.4.2**  
  - If you move cables, run `./monitor_camera_move.sh` to see current by-path devices and update the `CAMERA_DEVICES` array in `set_camera_fps.sh` (and the default in `launch_cv.py` if you change it).
- **Scripts kept:**  
  - `comp.sh` — full competition run (3 cameras, all tasks, planner): MAVROS, global_frame, LiDAR, CV, fusion, planner.  
  - `comp_single_camera.sh` — single camera, all tasks, planner. Override: `CAMERA1_DEVICE=/dev/video0` or by-path.
- `comp_single_camera_task3.sh` — Task 3 specific (indicator buoy, no task4/number).
- `build.sh` — build the workspace.  
  - `monitor_camera_move.sh` — shows where each camera is plugged in (by-path), no “move to USB 3.0” logic.

## Bandwidth / USB

- For multiple cameras, the kernel USB buffer can be too small. If you see USB errors or dropouts, increase usbfs memory, e.g.:
  - **Temporary:** `echo 2000 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`
  - **Persistent:** add `usbcore.usbfs_memory_mb=2000` to kernel cmdline (e.g. in `/boot/extlinux/extlinux.conf` on Jetson, then reboot).

## If errors show up again

- **“Camera X not found” / wrong camera order:** Run `./monitor_camera_move.sh` and align the listed by-path ports with Cam0/1/2 in `computer_vision/set_camera_fps.sh`. Override: `CAMERA_DEVICES="/path1,/path2,/path3" ./comp.sh` (3 cameras) or `CAMERA1_DEVICE=/dev/video0` for single camera.
- **Stale / no data on one camera:** Check USB connection and hub power. Ensure only one process opens each camera (no other `v4l2` or ROS nodes on the same device).
- **Permission denied on v4l2 control (e.g. control 10092545):** Usually non-fatal; streaming still works. To fix, add udev rules or run the node with sufficient privileges.
- **Launch:** Camera devices default is set in `computer_vision/src/cv_ros_nodes/cv_ros_nodes/launch/launch_cv.py` to match `set_camera_fps.sh`; override with `camera_devices:=/path1,/path2,/path3` if you change wiring.
