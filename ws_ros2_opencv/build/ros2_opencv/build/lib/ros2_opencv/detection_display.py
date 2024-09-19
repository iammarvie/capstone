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
            3)
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()

          # Initialize video writer
        self.video_writer = None
        self.video_file = 'output_video.avi'  # Output video file name
        self.frame_width = 640  # Adjust to match your frame width
        self.frame_height = 480  # Adjust to match your frame height
        self.frame_rate = 20.0  # Frames per second

    def listener_callback(self, msg):
        
        self.get_logger().info('Received an image on detection_image')
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        

         # Initialize video writer if not already initialized
        if self.video_writer is None:
            # Define the codec and create VideoWriter object
            fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec, e.g., 'XVID' or 'MJPG'
            self.video_writer = cv2.VideoWriter(self.video_file, fourcc, self.frame_rate, (self.frame_width, self.frame_height))

        # Write the frame to the video file
        self.video_writer.write(cv_image)

        # Display the image
        cv2.imshow('Detected Image', cv_image)
        cv2.waitKey(1)

        def destroy_node(self):
        # Release the video writer object
            if self.video_writer is not None:
                self.video_writer.release()
            cv2.destroyAllWindows()
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
