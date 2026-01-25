"""
Task 4 Supply Drop Processor ROS2 Node.

Runs yellow/black blob detection in parallel with main YOLO inference,
matches shape detections (triangle=8, cross=6) from per-camera /camera{N}/detection_info
to blobs via ROI-above-blob logic, and publishes supply drop targets per camera
to /camera{N}/task4_detections for the combiner.

Subscribes to: /camera{N}/image_preprocessed, /camera{N}/detection_info (N=0,1,2)
Publishes to: /camera{N}/task4_detections (String JSON), N=0,1,2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import time
from typing import List, Dict, Tuple

from cv_ros_nodes.task4_blob_utils import (
    find_yellow_blobs,
    find_black_blobs,
    roi_above_blob,
)

# Class IDs from main model (class_mapping.yaml)
CLASS_CROSS = 6
CLASS_TRIANGLE = 8

# Inference outputs in 640x640; preprocessed and blob/ROI are 640x480.
# cv2.resize(640x480 -> 640x640) stretches y by 640/480, so: y_prep = y * 480/640
Y_SCALE_PREP = 480.0 / 640.0


class Task4SupplyProcessor(Node):
    def __init__(self):
        super().__init__('task4_supply_processor')
        self.bridge = CvBridge()

        # {camera_id: {'yellow': [(roi, blob), ...], 'black': [(roi, blob), ...]}}
        self._blobs_by_camera: Dict[int, Dict[str, List[Tuple[Tuple[int, int, int, int], Tuple[int, int, int, int, float]]]]] = {}

        # Subscriptions: preprocessed images for blob detection
        for cam_id in [0, 1, 2]:
            self.create_subscription(
                Image,
                f'/camera{cam_id}/image_preprocessed',
                lambda msg, cid=cam_id: self._image_callback(msg, cid),
                10,
            )

        # Subscriptions: per-camera inference for shape (6, 8)
        for cam_id in [0, 1, 2]:
            self.create_subscription(
                String,
                f'/camera{cam_id}/detection_info',
                lambda msg, cid=cam_id: self._detection_callback(msg, cid),
                10,
            )

        # Publishers: per-camera task4 detections (for combiner)
        self._task4_pubs = {}
        for cam_id in [0, 1, 2]:
            self._task4_pubs[cam_id] = self.create_publisher(
                String, f'/camera{cam_id}/task4_detections', 10
            )

        self.get_logger().info('Task 4 supply processor initialized')
        self.get_logger().info('Subscribed to: /camera{N}/image_preprocessed, /camera{N}/detection_info')
        self.get_logger().info('Publishing to: /camera{N}/task4_detections')

    def _image_callback(self, msg: Image, camera_id: int) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'image_callback camera{camera_id}: {e}')
            return

        yellow_blobs, _ = find_yellow_blobs(bgr, min_area=300)
        black_blobs, _ = find_black_blobs(bgr, min_area=2000)

        yellow_roi_blob: List[Tuple[Tuple[int, int, int, int], Tuple[int, int, int, int, float]]] = []
        for b in yellow_blobs[:10]:
            roi = roi_above_blob(bgr.shape, b, above_scale=3.0, pad=50)
            if roi is not None:
                yellow_roi_blob.append((roi, b))

        black_roi_blob: List[Tuple[Tuple[int, int, int, int], Tuple[int, int, int, int, float]]] = []
        for b in black_blobs[:10]:
            roi = roi_above_blob(bgr.shape, b, above_scale=3.0, pad=50)
            if roi is not None:
                black_roi_blob.append((roi, b))

        self._blobs_by_camera[camera_id] = {
            'yellow': yellow_roi_blob,
            'black': black_roi_blob,
        }

    def _detection_callback(self, msg: String, camera_id: int) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'detection_callback camera{camera_id}: invalid JSON: {e}')
            return

        detections = data.get('detections', [])
        shapes = [d for d in detections if d.get('class_id') in (CLASS_CROSS, CLASS_TRIANGLE)]

        out: List[Dict] = []
        timestamp = data.get('timestamp', time.time())

        blobs = self._blobs_by_camera.get(camera_id)
        if not blobs or (not blobs.get('yellow') and not blobs.get('black')):
            self._publish_task4(camera_id, timestamp, out)
            return

        for d in shapes:
            # Bbox from detection_info (already in preprocessed frame); prefer bbox list, else x1,y1,x2,y2
            bbox = d.get('bbox')
            if bbox and len(bbox) == 4:
                x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])
            else:
                x1 = float(d.get('x1', 0))
                y1 = float(d.get('y1', 0))
                x2 = float(d.get('x2', 0))
                y2 = float(d.get('y2', 0))

            shape_bbox = [x1, y1, x2, y2]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            class_id = int(d.get('class_id', -1))
            score = float(d.get('score', 0.0))

            def in_roi(r: Tuple[int, int, int, int]) -> bool:
                rx1, ry1, rx2, ry2 = r
                return rx1 <= cx <= rx2 and ry1 <= cy <= ry2

            if class_id == CLASS_TRIANGLE:
                for roi, blob in blobs.get('yellow', []):
                    if in_roi(roi):
                        x, y, w, h, _ = blob
                        vessel_bbox = [x, y, x + w, y + h]
                        out.append({
                            'type': 'yellow_supply_drop',
                            'class_id': CLASS_TRIANGLE,
                            'score': score,
                            'shape_bbox': shape_bbox,
                            'vessel_bbox': vessel_bbox,
                            'source': 'task4',
                        })
                        break

            elif class_id == CLASS_CROSS:
                for roi, blob in blobs.get('black', []):
                    if in_roi(roi):
                        x, y, w, h, _ = blob
                        vessel_bbox = [x, y, x + w, y + h]
                        out.append({
                            'type': 'black_supply_drop',
                            'class_id': CLASS_CROSS,
                            'score': score,
                            'shape_bbox': shape_bbox,
                            'vessel_bbox': vessel_bbox,
                            'source': 'task4',
                        })
                        break

        self._publish_task4(camera_id, timestamp, out)

    def _publish_task4(self, camera_id: int, timestamp: float, detections: List[Dict]) -> None:
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
