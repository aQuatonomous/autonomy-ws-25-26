#!/usr/bin/env python3
"""
Echo planning-related ROS2 topics to the terminal.
Run in a separate terminal while the comp runs in another.
Subscribes to: /planned_path (with full global waypoint set), /curr_task,
/gs_message_send, /messages/gate_pass, cmd_vel.
"""

import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32


# Default cmd_vel topic used by comp scripts (task_test_comp, task1_comp, etc.)
DEFAULT_CMD_VEL_TOPIC = "/uas1/mavros/setpoint_velocity/cmd_vel_unstamped"


def path_summary(msg: Path) -> str:
    n = len(msg.poses) if msg.poses else 0
    if n == 0:
        return "0 waypoints"
    return f"{n} waypoints"


MAX_WAYPOINTS_ECHO = 100  # cap so terminal doesn't flood on long paths


def path_waypoints(msg: Path) -> str:
    """Global waypoint set: (x, y) for each pose; capped at MAX_WAYPOINTS_ECHO."""
    if not msg.poses:
        return "[]"
    pts = []
    for i, p in enumerate(msg.poses):
        if i >= MAX_WAYPOINTS_ECHO:
            pts.append(f"... +{len(msg.poses) - MAX_WAYPOINTS_ECHO} more")
            break
        x, y = p.pose.position.x, p.pose.position.y
        pts.append(f"({x:.1f},{y:.1f})")
    return " ".join(pts)


def twist_summary(msg: Twist) -> str:
    return f"linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}) angular=({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})"


class PlanningEchoNode(Node):
    def __init__(self, cmd_vel_topic: str):
        super().__init__("planning_echo")
        self._cmd_vel_topic = cmd_vel_topic

        self.create_subscription(Path, "/planned_path", self._planned_path_cb, 10)
        self.create_subscription(Int32, "/curr_task", self._curr_task_cb, 10)
        self.create_subscription(String, "/gs_message_send", self._gs_message_cb, 10)
        self.create_subscription(String, "/messages/gate_pass", self._gate_pass_cb, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._cmd_vel_cb, 10)

        self.get_logger().info(
            f"Echoing planning topics: /planned_path, /curr_task, /gs_message_send, "
            f"/messages/gate_pass, {self._cmd_vel_topic}"
        )

    def _planned_path_cb(self, msg: Path) -> None:
        print(f"[planned_path] {path_summary(msg)}: {path_waypoints(msg)}")

    def _curr_task_cb(self, msg: Int32) -> None:
        print(f"[curr_task] {msg.data}")

    def _gs_message_cb(self, msg: String) -> None:
        print(f"[gs_message_send] {msg.data}")

    def _gate_pass_cb(self, msg: String) -> None:
        print(f"[gate_pass] {msg.data}")

    def _cmd_vel_cb(self, msg: Twist) -> None:
        print(f"[cmd_vel] {twist_summary(msg)}")


def main():
    cmd_vel_topic = DEFAULT_CMD_VEL_TOPIC
    if len(sys.argv) > 1:
        cmd_vel_topic = sys.argv[1]

    rclpy.init()
    node = PlanningEchoNode(cmd_vel_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
