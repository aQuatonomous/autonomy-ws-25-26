"""
Task 4 Supply Drop Processor ROS2 Node.

Runs the simplified blob detection + shape verification pipeline directly on each
preprocessed camera frame. No YOLO shape detection dependency.

Subscribes to: /camera{N}/image_preprocessed  (N = 0, 1, 2)
Publishes to:  /camera{N}/task4_detections     (String JSON)

Detection format published:
    {
        "camera_id": <int>,
        "timestamp": <float>,
        "detections": [
            {
                "type":        "yellow_supply_drop" | "black_supply_drop",
                "vessel_bbox": [x1, y1, x2, y2],
                "confidence":  <float>,
                "has_shape":   <bool>,
                "source":      "task4"
            },
            ...
        ]
    }

Only detections with confidence >= 0.65 are included (enforced by the detector).
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

from cv_ros_nodes.task4_simplified_detector import SimplifiedTask4Detector


class Task4SupplyProcessor(Node):
    def __init__(self):
        super().__init__('task4_supply_processor')
        self.bridge = CvBridge()
        self._detector = SimplifiedTask4Detector()

        # One subscription + publisher per camera
        self._task4_pubs = {}
        for cam_id in [0, 1, 2]:
            self.create_subscription(
                Image,
                f'/camera{cam_id}/image_preprocessed',
                lambda msg, cid=cam_id: self._image_callback(msg, cid),
                10,
            )
            self._task4_pubs[cam_id] = self.create_publisher(
                String, f'/camera{cam_id}/task4_detections', 10
            )

        self.get_logger().info('Task 4 supply processor initialized (simplified pipeline)')
        self.get_logger().info('Subscribed to: /camera{N}/image_preprocessed')
        self.get_logger().info('Publishing to: /camera{N}/task4_detections')
        self.get_logger().info('Confidence threshold: 0.65')

    def _image_callback(self, msg: Image, camera_id: int) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'image_callback camera{camera_id}: {e}')
            return

        try:
            detections = self._detector.process_frame(bgr)
        except Exception as e:
            self.get_logger().error(f'detection camera{camera_id}: {e}')
            detections = []

        self._publish_task4(camera_id, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            detections)

    def _publish_task4(self, camera_id: int, timestamp: float, detections: list) -> None:
        payload = {
            'camera_id': camera_id,
            'timestamp': timestamp,
            'detections': detections,
        }
        m = String()
        m.data = json.dumps(payload)
        self._task4_pubs[camera_id].publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = Task4SupplyProcessor()
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


if __name__ == '__main__':
    main()
