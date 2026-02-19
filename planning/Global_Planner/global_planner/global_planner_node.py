#!/usr/bin/env python3
"""
Global planner ROS2 node.

Subscribes to:
  - /fused_buoys (FusedBuoyArray) from cv_lidar_fusion
  - /odom (nav_msgs/Odometry)

Maintains a persistent EntityList, runs TaskMaster (Task1/2/3) at 10 Hz,
publishes /planned_path (nav_msgs/Path) and /mavros/setpoint_velocity/cmd_vel_unstamped
(geometry_msgs/Twist) for MAVROS.

All planning library code (entities, Task1/2/3, TaskMaster, potential_fields_planner)
remains ROS-free; this node is the only place with rclpy.
"""

from __future__ import annotations

import os
import sys
from typing import Optional

# Ensure planning library is on path (sibling of global_planner package in workspace)
def _add_planning_path():
    planning_path = os.environ.get("PLANNING_PATH")
    if not planning_path or not os.path.isdir(planning_path):
        # Try relative to this file: .../install/global_planner/lib/python3.x/site-packages/global_planner
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("global_planner")
            # share = .../install/global_planner/share/global_planner -> src = .../src
            ws_src = os.path.abspath(os.path.join(share, "..", "..", "..", "src"))
            planning_path = os.path.join(ws_src, "planning")
        except Exception:
            planning_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "planning"))
    if os.path.isdir(planning_path) and planning_path not in sys.path:
        sys.path.insert(0, planning_path)


_add_planning_path()

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String as StringMsg
from pointcloud_filters.msg import FusedBuoyArray
import math

try:
    from mavros_msgs.msg import State as MavrosState
    _MAVROS_STATE_AVAILABLE = True
except ImportError:
    _MAVROS_STATE_AVAILABLE = False
    MavrosState = None

# Planning library (no ROS)
from Global.entities import EntityList, apply_tracked_buoys, entity_list_to_detections
from TaskMaster import TaskMaster, DetectedEntity

from .watchdogs import tick as watchdog_tick, initial_state as watchdog_initial_state


def _log_prefix(clock, pose) -> str:
    """Prefix for all boat logs: timestamp and boat position (x, y) in m."""
    t = clock.now()
    sec = t.nanoseconds * 1e-9
    x, y = float(pose[0]), float(pose[1])
    return f"[t={sec:.1f}s] x={x:.1f} y={y:.1f} (m) | "


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner_node")

        self.declare_parameter("task_id", 1)
        self.declare_parameter("planning_hz", 10.0)
        self.declare_parameter("cmd_vel_topic", "/mavros/setpoint_velocity/cmd_vel_unstamped")
        self.declare_parameter("use_odom_frame_for_path", True)

        task_id = int(self.get_parameter("task_id").value)
        self._planning_hz = float(self.get_parameter("planning_hz").value)
        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self._entity_list = EntityList(start_position=(0.0, 0.0))
        self._latest_pose = (0.0, 0.0, 0.0)  # x, y, heading (m, m, rad)
        self._latest_velocity_m_s = (0.0, 0.0)  # world frame (odom), from Pixhawk twist, m/s
        self._odom_header = None
        self._buoy_count = 0
        self._start_set = False

        # Guided-mode gate: only run planning when Pixhawk is in GUIDED
        self._guided_mode_active = not _MAVROS_STATE_AVAILABLE  # allow planning if no mavros_msgs
        if _MAVROS_STATE_AVAILABLE:
            self._mavros_state_sub = self.create_subscription(
                MavrosState,
                "/mavros/state",
                self._mavros_state_callback,
                10,
            )
        else:
            self.get_logger().warn("mavros_msgs not available; guided-mode gate disabled, planning allowed")

        # Global reference for lat/long (Task 2 debris report). Set from first NavSatFix + odom.
        self._global_ref = None  # (lat_ref, lon_ref, x_ref_m, y_ref_m)
        # Task 2 debris report throttle (avoid publishing every tick)
        self._last_debris_report_time: Optional[float] = None
        self._debris_report_interval_sec = 2.0
        # Task 3 indicator color report throttle (same topic /gs_message_send)
        self._last_indicator_report_time: Optional[float] = None
        self._indicator_report_interval_sec = 2.0

        self._task_master = TaskMaster(
            entities=self._entity_list,
            start=(0.0, 0.0),
            task_id=task_id,
            map_bounds=None,
        )

        self._fused_sub = self.create_subscription(
            FusedBuoyArray,
            "/fused_buoys",
            self._fused_callback,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            10,
        )

        self._path_pub = self.create_publisher(Path, "/planned_path", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._gs_message_pub = self.create_publisher(StringMsg, "/gs_message_send", 10)

        self._global_position_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._global_position_callback,
            10,
        )

        self._watchdog_state = watchdog_initial_state()
        self._timer = self.create_timer(1.0 / self._planning_hz, self._planning_tick)

        self.get_logger().info(
            f"global_planner_node started: task_id={task_id}, hz={self._planning_hz}, "
            f"cmd_vel={self._cmd_vel_topic}"
        )

    def _mavros_state_callback(self, msg) -> None:
        self._guided_mode_active = (getattr(msg, "mode", "") == "GUIDED")

    def _global_position_callback(self, msg: NavSatFix) -> None:
        if self._global_ref is not None:
            return
        if not self._start_set:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        x_m = float(self._latest_pose[0])
        y_m = float(self._latest_pose[1])
        self._global_ref = (float(msg.latitude), float(msg.longitude), x_m, y_m)
        self.get_logger().info(
            f"Global reference set: lat={self._global_ref[0]:.6f} lon={self._global_ref[1]:.6f} "
            f"(odom x={x_m:.2f} y={y_m:.2f} m)"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        heading = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._latest_pose = (x, y, heading)
        self._odom_header = msg.header
        # Twist is in child (body) frame; rotate to world (odom) for watchdog.
        vx_b = float(msg.twist.twist.linear.x)
        vy_b = float(msg.twist.twist.linear.y)
        c, s = math.cos(heading), math.sin(heading)
        vx_w = vx_b * c - vy_b * s
        vy_w = vx_b * s + vy_b * c
        self._latest_velocity_m_s = (float(vx_w), float(vy_w))
        if not self._start_set:
            self._entity_list.set_start_position((x, y))
            self._start_set = True
            self.get_logger().info(f"Start position set from odom: ({x:.1f}, {y:.1f}) m")

    def _fused_callback(self, msg: FusedBuoyArray) -> None:
        n = len(msg.buoys)
        pose = self._latest_pose
        prefix = _log_prefix(self.get_clock(), pose)
        # Positions in m; pass class_id when message has it (fusion), else color (legacy)
        buoys_m = []
        for b in msg.buoys:
            item = {"id": b.id, "x": float(b.x), "y": float(b.y)}
            if getattr(b, "class_id", None) is not None:
                item["class_id"] = b.class_id
            else:
                item["color"] = getattr(b, "color", "unknown")
            buoys_m.append(item)
        apply_tracked_buoys(self._entity_list, buoys_m, tolerance=1.0)
        self._buoy_count = n
        gates = self._entity_list.get_gates()
        obstacles = self._entity_list.get_obstacles()
        types_seen = set()
        for b in msg.buoys:
            if getattr(b, "class_id", None) is not None:
                types_seen.add(f"class_{b.class_id}")
            else:
                types_seen.add(getattr(b, "color", "unknown"))
        self.get_logger().info(
            f"{prefix}Buoy update: received {n} buoys, types={types_seen}, "
            f"gates={len(gates)}, obstacles={len(obstacles)}"
        )
        self.get_logger().info(
            f"{prefix}Seen red-green buoy pairs: {len(gates)} pair(s) "
            f"{[(f'red@({r[0]:.1f},{r[1]:.1f}) green@({g[0]:.1f},{g[1]:.1f})') for r, g in gates]}"
        )
        if getattr(self._entity_list, "_last_pruned_ids", None):
            dropped = self._entity_list._last_pruned_ids
            if dropped:
                self.get_logger().info(f"{prefix}Dropped entities (cohort prune): ids={sorted(dropped)}")

    def _get_pose(self):
        return self._latest_pose

    def _get_detections(self):
        out = []
        for bid, etype, pos in entity_list_to_detections(self._entity_list):
            out.append(DetectedEntity(entity_id=bid, entity_type=etype, position=pos))
        return out

    def _odom_to_latlon(self, x_m: float, y_m: float):
        """Convert odom position (m) to (lat, lon) using stored global reference. Returns (lat, lon) or None."""
        if self._global_ref is None:
            return None
        lat_ref, lon_ref, x_ref_m, y_ref_m = self._global_ref
        # ENU: x East, y North. meters to degrees (approx)
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(lat_ref))
        lat = lat_ref + (y_m - y_ref_m) / m_per_deg_lat
        lon = lon_ref + (x_m - x_ref_m) / m_per_deg_lon
        return (lat, lon)

    def _planning_tick(self) -> None:
        pose = self._get_pose()
        prefix = _log_prefix(self.get_clock(), pose)

        if not self._guided_mode_active:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)
            return

        # (4) Skip planning until we have received at least one odom (real pose/start).
        if not self._start_set:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)
            return

        try:
            # Map bounds from no-go zones (same everywhere)
            map_bounds = getattr(self._entity_list, "get_map_bounds", lambda: None)()
            result = self._task_master.run_one_shot(
                get_pose=self._get_pose,
                get_detections=self._get_detections,
                use_planning=True,
                map_bounds=map_bounds,
            )

            path = result.get("path") or []
            velocities = getattr(self._task_master.manager, "current_velocities", None)
            speeds = getattr(self._task_master.manager, "current_speeds", None)
            goals = self._entity_list.get_goals()
            gates = self._entity_list.get_gates()
            no_go_pts = self._entity_list.get_no_go_obstacle_points(map_bounds=map_bounds)

            # Current task
            task_id = self._task_master.task_id
            self.get_logger().info(f"{prefix}Current task: task_id={task_id}")

            # Seen red-green buoy pairs
            self.get_logger().info(
                f"{prefix}Seen red-green buoy pairs: {len(gates)} "
                f"{[(f'({r[0]:.1f},{r[1]:.1f})-({g[0]:.1f},{g[1]:.1f})') for r, g in gates]}"
            )

            # Goal / next goal
            if goals:
                next_goal = goals[0]
                self.get_logger().info(
                    f"{prefix}Seen goal / next goal: count={len(goals)}, next=({next_goal[0]:.1f}, {next_goal[1]:.1f}) m"
                )
            else:
                self.get_logger().info(f"{prefix}No goal seen")

            # No-go zone
            if no_go_pts:
                self.get_logger().info(f"{prefix}No-go zone identified: {len(no_go_pts)} boundary points")
            else:
                self.get_logger().info(f"{prefix}No-go zone: none (no gate pairs or using cached)")

            # State: following goal vs going straight / prediction
            has_vel = velocities is not None and len(velocities) > 0 and speeds is not None and len(speeds) > 0
            if goals and has_vel:
                self.get_logger().info(f"{prefix}State: following goal")
            elif not goals:
                self.get_logger().info(f"{prefix}State: no goal (idle or going straight)")
            else:
                self.get_logger().info(f"{prefix}State: goal set but no velocity (waiting or stuck)")

            # Current entity list (summary by type and id)
            by_type = {}
            for e in self._entity_list.entities:
                if e.type == "goal":
                    continue
                key = e.type
                if key not in by_type:
                    by_type[key] = []
                eid = e.entity_id if e.entity_id is not None else "?"
                by_type[key].append(f"id={eid}@({e.position[0]:.1f},{e.position[1]:.1f})")
            self.get_logger().info(f"{prefix}Current entity list: {by_type}")

            # Dropped entities (from last cohort prune)
            dropped = getattr(self._entity_list, "_last_pruned_ids", set()) or set()
            if dropped:
                self.get_logger().info(f"{prefix}Dropped entities: ids={sorted(dropped)}")

            # Task 2: report lat/long of black buoys (debris) and green/red indicators to /gs_message_send (throttled)
            if task_id == 2 and self._global_ref is not None:
                time_now_sec = self.get_clock().now().nanoseconds * 1e-9
                if self._last_debris_report_time is None or (time_now_sec - self._last_debris_report_time) >= self._debris_report_interval_sec:
                    # Black buoys (debris) - same format as before
                    black_buoys = self._entity_list.get_black_buoys()
                    if black_buoys:
                        parts = []
                        for eid, pos in black_buoys:
                            ll = self._odom_to_latlon(float(pos[0]), float(pos[1]))
                            if ll is not None:
                                lat, lon = ll
                                parts.append(f"{eid},{lat:.6f},{lon:.6f}")
                        if parts:
                            self._gs_message_pub.publish(StringMsg(data="DEBRIS_LATLON|" + "|".join(parts)))
                    # Green and red indicators - same style (id,lat,lon) stored/sent like black buoys
                    green_ents = self._entity_list.get_by_type("green_indicator")
                    red_ents = self._entity_list.get_by_type("red_indicator")
                    ind_parts = []
                    for e in green_ents:
                        ll = self._odom_to_latlon(float(e.position[0]), float(e.position[1]))
                        if ll is not None:
                            lat, lon = ll
                            eid = e.entity_id if e.entity_id is not None else 0
                            ind_parts.append(f"green,{eid},{lat:.6f},{lon:.6f}")
                    for e in red_ents:
                        ll = self._odom_to_latlon(float(e.position[0]), float(e.position[1]))
                        if ll is not None:
                            lat, lon = ll
                            eid = e.entity_id if e.entity_id is not None else 0
                            ind_parts.append(f"red,{eid},{lat:.6f},{lon:.6f}")
                    if ind_parts:
                        self._gs_message_pub.publish(StringMsg(data="INDICATOR_LATLON|" + "|".join(ind_parts)))
                    self._last_debris_report_time = time_now_sec

            # Task 3: report indicator color to /gs_message_send (same topic as Task 2, throttled)
            if task_id == 3:
                indicator_color = getattr(self._task_master.manager, "indicator_color", None)
                if indicator_color is not None:
                    time_now_sec = self.get_clock().now().nanoseconds * 1e-9
                    if self._last_indicator_report_time is None or (time_now_sec - self._last_indicator_report_time) >= self._indicator_report_interval_sec:
                        msg_str = f"INDICATOR_COLOR|{indicator_color}"
                        self._gs_message_pub.publish(StringMsg(data=msg_str))
                        self._last_indicator_report_time = time_now_sec

            # Next path given and velocity given
            if goals:
                next_wp = goals[0]
                self.get_logger().info(
                    f"{prefix}Next path (waypoint) given: ({next_wp[0]:.1f}, {next_wp[1]:.1f}) m"
                )
            else:
                self.get_logger().info(f"{prefix}Next path given: none")

            if has_vel:
                vx = float(velocities[0][0])
                vy = float(velocities[0][1])
                speed = float(speeds[0])
                self.get_logger().info(
                    f"{prefix}Velocity given: vx={vx:.2f} vy={vy:.2f} m/s speed={speed:.2f} m/s"
                )
            else:
                self.get_logger().info(f"{prefix}Velocity given: zero (no path)")

            # Watchdog: use actual velocity from Pixhawk (odom twist, world frame) for collision check.
            time_now = self.get_clock().now().nanoseconds * 1e-9
            dt_sec = 1.0 / self._planning_hz
            velocity_m_s = self._latest_velocity_m_s
            obstacles_m = list(self._entity_list.get_obstacles()) + list(no_go_pts)
            self._watchdog_state, twist_override, watchdog_reason = watchdog_tick(
                self._watchdog_state,
                time_now,
                dt_sec,
                (pose[0], pose[1], pose[2]),
                velocity_m_s,
                obstacles_m,
                bool(goals),
            )
            if twist_override is not None and watchdog_reason:
                self.get_logger().info(f"{prefix}This is the watchdog protocol happening. {watchdog_reason}")
                if "spinning" in watchdog_reason.lower():
                    self.get_logger().info(f"{prefix}We are spinning.")
                twist = Twist()
                twist.linear.x = float(twist_override.get("linear.x", 0.0))
                twist.linear.y = float(twist_override.get("linear.y", 0.0))
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = float(twist_override.get("angular.z", 0.0))
                self._cmd_vel_pub.publish(twist)
                return
            # Normal: use planner output
            if goals:
                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id = self._odom_header.frame_id if self._odom_header else "odom"
                next_wp = goals[0]
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = float(next_wp[0])
                ps.pose.position.y = float(next_wp[1])
                ps.pose.position.z = 0.0
                path_msg.poses.append(ps)
                self._path_pub.publish(path_msg)

            twist = Twist()
            if has_vel:
                # Planner gives world-frame velocity (m/s). Convert to body frame for MAVROS.
                vx_w = float(velocities[0][0])
                vy_w = float(velocities[0][1])
                hdg = float(pose[2])
                c, s = math.cos(hdg), math.sin(hdg)
                v_forward = vx_w * c + vy_w * s
                v_left = -vx_w * s + vy_w * c
                twist.linear.x = v_forward
                twist.linear.y = v_left
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                self._cmd_vel_pub.publish(twist)
            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                self._cmd_vel_pub.publish(twist)

            if result.get("status") == "SUCCESS":
                self.get_logger().info(f"{prefix}Task completed successfully")

        except Exception as e:
            self.get_logger().error(f"Planning tick error: {e}")
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._cmd_vel_pub.publish(twist)

    def destroy_node(self) -> None:
        self._timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
