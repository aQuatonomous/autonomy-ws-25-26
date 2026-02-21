# Roboboat 2026 - Autonomy Workspace

Welcome to the **Autonomy** repository for the Roboboat 2026 project.

## Current setup (this session)

- **Cameras:** 3× Arducam B0495 (USB3 2.3MP), on a powered USB hub (e.g. TP-Link UH700).  
  Device paths are set in **one place**: `set_camera_fps.sh` (repo root).  
  - Single camera: default port **1.4.2**. Three cameras: Cam0 = **1.2**, Cam1 = **1.1**, Cam2 = **1.4.2**.  
  - If you move cables, run `./monitor_camera_move.sh` and edit the defaults in `set_camera_fps.sh`.
- **Scripts kept:**  
  - `comp.sh` — full competition run (3 cameras, all tasks, planner): MAVROS, global_frame, LiDAR, CV, fusion, planner.  
  - `comp_single_camera.sh` — single camera, all tasks, planner (uses `./set_camera_fps.sh single`).
- `comp_single_camera_task3.sh` — Task 3 specific (indicator buoy, no task4/number).
- `build.sh` — build the workspace.  
  - `monitor_camera_move.sh` — shows where each camera is plugged in (by-path), no “move to USB 3.0” logic.

## Bandwidth / USB

- For multiple cameras, the kernel USB buffer can be too small. If you see USB errors or dropouts, increase usbfs memory, e.g.:
  - **Temporary:** `echo 2000 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`
  - **Persistent:** add `usbcore.usbfs_memory_mb=2000` to kernel cmdline (e.g. in `/boot/extlinux/extlinux.conf` on Jetson, then reboot).

## If errors show up again

- **“Camera X not found” / wrong camera order:** Run `./monitor_camera_move.sh` and edit device paths in `set_camera_fps.sh` (repo root). Override: `CAMERA_DEVICES="/path1,/path2,/path3" ./set_camera_fps.sh three` then run the comp script.
- **Stale / no data on one camera:** Check USB connection and hub power. Ensure only one process opens each camera (no other `v4l2` or ROS nodes on the same device).
- **Permission denied on v4l2 control (e.g. control 10092545):** Usually non-fatal; streaming still works. To fix, add udev rules or run the node with sufficient privileges.
- **Launch:** Comp scripts call `./set_camera_fps.sh single` or `three` and pass the device list to the CV launch; device paths live only in `set_camera_fps.sh`.
