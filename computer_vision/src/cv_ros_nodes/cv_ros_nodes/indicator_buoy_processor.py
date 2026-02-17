"""
Indicator Buoy Processor ROS2 Node.

Subscribes to: /camera{N}/image_preprocessed (N=0,1,2)
Publishes to: /camera{N}/indicator_detections (String JSON)

Runs the colour indicator buoy pipeline (diamonds + red/green indicator) on each
preprocessed image and publishes detections for the combiner. When enable_indicator_buoy
is true, these appear in /combined/detection_info with class_name red_indicator_buoy
or green_indicator_buoy.
"""

import json
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# Add task_specific to path so we can import colour_indicator_buoy_detector
_this_dir = Path(__file__).resolve().parent
# When run from source: .../computer_vision/src/cv_ros_nodes/cv_ros_nodes -> .../computer_vision/task_specific
_task_specific_src = _this_dir.parent.parent.parent / "task_specific"
_task_specific_ws = Path(os.path.expanduser("~")) / "autonomy-ws-25-26" / "computer_vision" / "task_specific"
_task_specific = _task_specific_src if _task_specific_src.is_dir() else _task_specific_ws
if _task_specific.is_dir() and str(_task_specific) not in sys.path:
    sys.path.insert(0, str(_task_specific))

# Class IDs for combined output (class_mapping.yaml)
CLASS_RED_INDICATOR_BUOY = 9
CLASS_GREEN_INDICATOR_BUOY = 10


def _whole_buoy_bbox(info: dict, img_shape) -> list:
    """Compute whole-buoy bbox as union of diamond boxes and indicator bbox. Returns [x1, y1, x2, y2]."""
    boxes = []
    for d in info.get("diamonds", []):
        b = d.get("bbox")
        if b and len(b) == 4:
            x, y, w, h = b
            boxes.append((x, y, x + w, y + h))
    ind = info.get("indicator_bbox")
    if ind and len(ind) == 4:
        x, y, w, h = ind
        boxes.append((x, y, x + w, y + h))
    if not boxes:
        return None
    h_img, w_img = img_shape[:2]
    x1 = max(0, min(b[0] for b in boxes))
    y1 = max(0, min(b[1] for b in boxes))
    x2 = min(w_img, max(b[2] for b in boxes))
    y2 = min(h_img, max(b[3] for b in boxes))
    if x1 >= x2 or y1 >= y2:
        return None
    return [float(x1), float(y1), float(x2), float(y2)]


class IndicatorBuoyProcessor(Node):
    def __init__(self, conf_threshold: float = 0.3, roi_conf_threshold: float = 0.6):
        super().__init__("indicator_buoy_processor")
        self.bridge = CvBridge()
        self.conf_threshold = conf_threshold
        self.roi_conf_threshold = roi_conf_threshold

        try:
            from task_2_3.colour_indicator_buoy_detector import classify_colour_indicator_buoy
            self._classify = classify_colour_indicator_buoy
        except ImportError as e:
            self.get_logger().error(f"Failed to import colour_indicator_buoy_detector: {e}")
            self._classify = None

        for cam_id in [0, 1, 2]:
            self.create_subscription(
                Image,
                f"/camera{cam_id}/image_preprocessed",
                lambda msg, cid=cam_id: self._image_callback(msg, cid),
                10,
            )
        self._pubs = {}
        for cam_id in [0, 1, 2]:
            self._pubs[cam_id] = self.create_publisher(
                String, f"/camera{cam_id}/indicator_detections", 10
            )

        self.get_logger().info("Indicator buoy processor initialized")
        self.get_logger().info("Subscribed to: /camera{N}/image_preprocessed")
        self.get_logger().info("Publishing to: /camera{N}/indicator_detections")

    def _image_callback(self, msg: Image, camera_id: int) -> None:
        if self._classify is None:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"image_callback camera{camera_id}: {e}")
            return

        try:
            _, info = self._classify(
                bgr,
                conf_threshold=self.conf_threshold,
                roi_conf_threshold=self.roi_conf_threshold,
            )
        except Exception as e:
            self.get_logger().error(f"classify_colour_indicator_buoy camera{camera_id}: {e}")
            self._publish(camera_id, time.time(), [])
            return

        state = info.get("indicator_state", "none")
        conf = float(info.get("indicator_conf", 0.0))
        detections = []

        if state in ("red", "green"):
            shape_bbox = _whole_buoy_bbox(info, bgr.shape)
            if shape_bbox:
                class_id = CLASS_RED_INDICATOR_BUOY if state == "red" else CLASS_GREEN_INDICATOR_BUOY
                detections.append({
                    "class_id": class_id,
                    "score": conf,
                    "shape_bbox": shape_bbox,
                    "indicator_color": state,
                    "source": "indicator_buoy",
                })

        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        self._publish(camera_id, timestamp, detections)

    def _publish(self, camera_id: int, timestamp: float, detections: list) -> None:
        payload = {
            "camera_id": camera_id,
            "timestamp": timestamp,
            "detections": detections,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pubs[camera_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    import argparse
    p = argparse.ArgumentParser(description="Indicator buoy processor")
    p.add_argument("--conf_threshold", type=float, default=0.3, help="Diamond confidence threshold")
    p.add_argument("--roi_conf_threshold", type=float, default=0.6, help="ROI diamond confidence threshold")
    parsed, _ = p.parse_known_args(args=args)
    node = IndicatorBuoyProcessor(
        conf_threshold=parsed.conf_threshold,
        roi_conf_threshold=parsed.roi_conf_threshold,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
