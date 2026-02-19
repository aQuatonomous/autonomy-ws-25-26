# MAVROS on Jetson — Quick Reference

MAVROS bridges your **Pixhawk** (or other ArduPilot flight controller) to ROS 2. This README explains how to run MAVROS on the Jetson and how to read common topics (GPS, bearing, etc.).

---

## 1. Run MAVROS on Jetson

### Requirements

- Pixhawk connected via USB (`/dev/ttyACM0` on Jetson)
- ROS 2 Humble and MAVROS installed

### Launch MAVROS

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600
```

If your Pixhawk is on a different port:

```bash
ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:57600
```

### Simulation (SITL)

For ArduPilot SITL on the same machine:

```bash
ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5760
```

---

## 2. Common MAVROS Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS: latitude, longitude, altitude (m) |
| `mavros/global_position/compass_hdg` | `std_msgs/Float64` | Compass heading (degrees): 0° = North, 90° = East |
| `mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Local pose (ENU: East, North, Up) |
| `mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | Local velocity (linear + angular) |
| `mavros/state` | `mavros_msgs/State` | Armed, mode, system status, connection |
| `mavros/battery` | `sensor_msgs/BatteryState` | Battery voltage, percentage |
| `mavros/global_position/rel_alt` | `std_msgs/Float64` | Relative altitude above home (m) |
| `mavros/imu/data` | `sensor_msgs/Imu` | IMU: orientation, angular velocity, linear accel |

---

## 3. How to Read Topics

### GPS (lat, lon, altitude)

```bash
ros2 topic echo /mavros/global_position/global
```

Example output:
```
header:
  stamp: ...
  frame_id: ''
latitude: 45.1234
longitude: -122.5678
altitude: 10.5
...
```

### Bearing / heading (degrees)

```bash
ros2 topic echo /mavros/global_position/compass_hdg
```

Example output:
```
data: 45.0   # 45° = NE
```

### Local position (ENU)

```bash
ros2 topic echo /mavros/local_position/pose
```

### Local velocity

```bash
ros2 topic echo /mavros/local_position/velocity_local
```

### State (armed, mode)

```bash
ros2 topic echo /mavros/state
```

Example output:
```
connected: true
armed: true
guided: false
mode: "MANUAL"
...
```

### List all MAVROS topics

```bash
ros2 topic list | grep mavros
```

---

## 4. Integration with this repo

`comp_single_camera.sh` and `comp.sh` already launch MAVROS when the Pixhawk device exists. They use:

- **FCU_URL**: `/dev/ttyACM0:57600` (override with `FCU_URL=... ./comp_single_camera.sh`)

The `global_frame` package subscribes to:

- `mavros/global_position/global` — GPS
- `mavros/global_position/compass_hdg` — heading

and publishes boat pose in a local map frame at `/boat_pose`.
