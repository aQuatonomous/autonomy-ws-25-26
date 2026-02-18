# MAVROS and boat state topics

## Serial ports

- **LiDAR:** `/dev/ttyUSB0` (Silicon Labs CP2104). Used by `buoy_pipeline.launch.py`.
- **Pixhawk:** `/dev/ttyACM0` (ArduPilot Pixhawk4). Used by MAVROS.  
  Stable by-id: `ls /dev/serial/by-id/` → `usb-ArduPilot_Pixhawk4_...-if00` → `/dev/ttyACM0`.

## How it runs

`./comp.sh` and `./comp_single_camera.sh` start MAVROS automatically when the Pixhawk is connected. Default: `fcu_url:=/dev/ttyACM0:57600`. Override with:

```bash
FCU_URL=/dev/ttyACM0:57600 ./comp_single_camera.sh
# or by-id (stable across reboots):
FCU_URL=/dev/serial/by-id/usb-ArduPilot_Pixhawk4_370020000A51333035363236-if00:57600 ./comp_single_camera.sh
```

If the serial device is missing, MAVROS is skipped and the rest of the pipeline still runs.

## Main topics (from MAVROS)

| Topic | Type | Description |
|-------|------|-------------|
| `mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS global position (lat/lon/alt). Indoors: no fix or invalid. |
| `mavros/global_position/compass_hdg` | `std_msgs/Float64` | Compass heading (degrees). |
| `mavros/imu/data` | `sensor_msgs/Imu` | IMU orientation, angular velocity, linear acceleration. Works indoors. |
| `mavros/imu/data_raw` | `sensor_msgs/Imu` | Raw IMU (no orientation). |
| `mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Local pose (when EKF has origin). |

The `global_frame` package subscribes to `mavros/global_position/global` and `mavros/global_position/compass_hdg` (see `mapping/src/global_frame/`).

## Simulation

For SITL, use TCP instead of serial: `FCU_URL:=tcp://localhost:5760` (see `simulations/README.md` and `simulation_full.bash`).
