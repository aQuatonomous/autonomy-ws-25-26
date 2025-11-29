import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageListener(Node):
    
    def __init__(self):
        super().__init__('image_listener') 
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera0/image_raw', 
            self.image_callback, 
            10
        )
        
    
    def image_callback(self,msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding= 'bgr8') # Convert ros2 image to cv2 image
        self.get_logger().info(f'Recieved image message of size {cv_image.shape}')
        
        
        #insert processing here
        
    
    

def main(args=None):
    rclpy.init(args=args) #initialize the ROS2 communication
    node = ImageListener() # create the node
    try:
        rclpy.spin(node) # spin the node until the node is stopped
    except KeyboardInterrupt:
        node.destroy_node() # destroy the node
    finally:
        rclpy.shutdown() # shutdown the ROS2 communication
        
if __name__ == '__main__':
    main()