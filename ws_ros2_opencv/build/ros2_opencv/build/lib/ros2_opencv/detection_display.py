import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageDisplayNode(Node):

    def __init__(self):
        super().__init__('image_display_node')
        self.subscription = self.create_subscription(
            Image,
            'detection_image',  # Topic to subscribe to
            self.listener_callback,
            20)
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        
        self.get_logger().info('Received an image on detection_image')
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Display the image
        cv2.imshow('Detected Image', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
