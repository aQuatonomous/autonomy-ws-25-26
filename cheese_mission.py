#!/usr/bin/env python3
"""
Evacuation mission: set current position as home, go 20 m ahead via MAVROS setpoint_position/global,
then return to home. Requires MAVROS running and FC in GUIDED (or position-control) mode.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geographic_msgs.msg import GeoPoseStamped, GeoPose, GeoPoint

# MAVROS GPS/compass often use BEST_EFFORT
_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# Approx metres per degree at mid-latitudes
M_PER_DEG_LAT = 111320.0
DIST_THRESH_M = 2.0
FORWARD_M = 9.0
SETPOINT_HZ = 10.0


def lat_lon_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Flat-earth distance in metres between two (lat, lon) points."""
    lat_rad = math.radians(lat1)
    dy = (lat2 - lat1) * M_PER_DEG_LAT
    dx = (lon2 - lon1) * M_PER_DEG_LAT * math.cos(lat_rad)
    return math.hypot(dx, dy)


def point_forward_m(lat: float, lon: float, heading_deg: float, distance_m: float) -> tuple[float, float]:
    """Return (lat, lon) of a point `distance_m` metres ahead of (lat, lon) given heading (deg, 0=North, 90=East)."""
    h = math.radians(heading_deg)
    d_north = distance_m * math.cos(h)
    d_east = distance_m * math.sin(h)
    d_lat = d_north / M_PER_DEG_LAT
    d_lon = d_east / (M_PER_DEG_LAT * math.cos(math.radians(lat)))
    return (lat + d_lat, lon + d_lon)


class CheeseMission(Node):
    def __init__(self):
        super().__init__("cheese_mission")
        self._home_lat: float | None = None
        self._home_lon: float | None = None
        self._home_alt: float = 0.0
        self._home_heading_deg: float | None = None
        self._current_lat: float | None = None
        self._current_lon: float | None = None
        self._got_fix = False
        self._got_heading = False

        self._global_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._on_global,
            _QOS,
        )
        self._heading_sub = self.create_subscription(
            Float64,
            "/mavros/global_position/compass_hdg",
            self._on_heading,
            _QOS,
        )
        self._setpoint_pub = self.create_publisher(
            GeoPoseStamped,
            "/mavros/setpoint_position/global",
            10,
        )
        self._timer = self.create_timer(1.0 / SETPOINT_HZ, self._timer_cb)
        self._phase = "wait_fix"  # wait_fix -> outbound -> return -> done
        self._target_lat: float | None = None
        self._target_lon: float | None = None

    def _on_global(self, msg: NavSatFix) -> None:
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude
        if not self._got_fix and msg.latitude != 0.0 and msg.longitude != 0.0:
            self._home_lat = msg.latitude
            self._home_lon = msg.longitude
            self._home_alt = getattr(msg, "altitude", 0.0) or 0.0
            self._got_fix = True

    def _on_heading(self, msg: Float64) -> None:
        if self._home_heading_deg is None:
            self._home_heading_deg = msg.data
            self._got_heading = True

    def _timer_cb(self) -> None:
        if self._phase == "done":
            return
        if self._phase == "wait_fix":
            if self._got_fix and self._got_heading:
                self._phase = "outbound"
                hdg = self._home_heading_deg or 0.0
                tlat, tlon = point_forward_m(
                    self._home_lat, self._home_lon, hdg, FORWARD_M
                )
                self._target_lat, self._target_lon = tlat, tlon
                self.get_logger().info(
                    f"Home set: ({self._home_lat:.6f}, {self._home_lon:.6f}). "
                    f"Target 20 m ahead: ({self._target_lat:.6f}, {self._target_lon:.6f})"
                )
            return
        if self._current_lat is None or self._current_lon is None:
            return

        if self._phase == "outbound":
            self._publish_setpoint(self._target_lat, self._target_lon, self._home_alt)
            d = lat_lon_distance_m(
                self._current_lat, self._current_lon,
                self._target_lat, self._target_lon,
            )
            if d <= DIST_THRESH_M:
                self.get_logger().info("Reached 20 m waypoint; returning to home.")
                self._phase = "return"
            return

        if self._phase == "return":
            self._publish_setpoint(self._home_lat, self._home_lon, self._home_alt)
            d = lat_lon_distance_m(
                self._current_lat, self._current_lon,
                self._home_lat, self._home_lon,
            )
            if d <= DIST_THRESH_M:
                self.get_logger().info("Returned to home. Mission complete.")
                self._phase = "done"
                self.create_timer(0.5, self._shutdown_cb)

    def _shutdown_cb(self) -> None:
        rclpy.shutdown()

    def _publish_setpoint(self, lat: float, lon: float, alt: float) -> None:
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose = GeoPose()
        msg.pose.position = GeoPoint()
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = lon
        msg.pose.position.altitude = alt
        self._setpoint_pub.publish(msg)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = CheeseMission()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
