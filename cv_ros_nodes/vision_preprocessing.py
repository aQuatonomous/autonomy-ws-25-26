import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse


class PreprocessCamera(Node):
    
    def __init__(self, camera_id: int):
        super().__init__(f'preprocess_camera{camera_id}')
        self.camera_id = camera_id
        self.bridge = CvBridge()
        self.frame_count = 0

        # Subscribe to raw camera images
        self.subscription = self.create_subscription(
            Image,
            f'/camera{camera_id}/image_raw',
            self.image_callback,
            10
        )

        # Publisher for processed images
        self.publisher = self.create_publisher(
            Image,
            f'/camera{camera_id}/image_preprocessed',
            10
        )
        
        self.get_logger().info(f'Preprocessing node for camera{camera_id} initialized')
        self.get_logger().info(f'Subscribed to: /camera{camera_id}/image_raw')
        self.get_logger().info(f'Publishing to: /camera{camera_id}/image_preprocessed')
        
    def image_callback(self, msg: Image):
        # 1) ROS2 Image -> OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2) Do some simple processing (example: resize)
        #    You can replace this with glare / hue / etc.
        processed = cv2.resize(cv_image, (640, 480))

        # 3) OpenCV -> ROS2 Image
        out_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')

        # Preserve original header (timestamp + frame_id)
        out_msg.header = msg.header

        # 4) Publish the processed image
        self.publisher.publish(out_msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Published 30 processed frames, last size={processed.shape}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 Camera Preprocessing Node')
    parser.add_argument('--camera_id', type=int, default=0,
                       help='Camera ID (0, 1, or 2)')
    args_parsed = parser.parse_args(args)
    
    if args_parsed.camera_id not in [0, 1, 2]:
        print("Error: camera_id must be 0, 1, or 2")
        return
    
    node = PreprocessCamera(camera_id=args_parsed.camera_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()

