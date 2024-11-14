import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageDisplayNode(Node):

    def __init__(self):
        super().__init__('image_display_node')

        # First subscription
        self.subscription1 = self.create_subscription(
            Image, 'lane_image', self.listener_callback1, 3
        )
        self.subscription1  # Prevent unused variable warning

        # Second subscription
        self.subscription2 = self.create_subscription(
            Image, 'canny_image', self.listener_callback2, 3
        )
        self.subscription2  # Prevent unused variable warning

        self.bridge = CvBridge()

        # Video writer for lane_image
        self.video_writer1 = None
        self.video_file1 = 'lane_output_video.avi'
        self.frame_width1 = 320
        self.frame_height1 = 320
        self.frame_rate1 = 5.0

        # Video writer for detection_image
        self.video_writer2 = None
        self.video_file2 = 'canny_ouput.avi'
        self.frame_width2 = 320
        self.frame_height2 = 320
        self.frame_rate2 = 5.0

    def listener_callback1(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Initialize video writer if not already initialized
        if self.video_writer1 is None:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer1 = cv2.VideoWriter(
                self.video_file1, fourcc, self.frame_rate1,
                (self.frame_width1, self.frame_height1)
            )

        # Write the frame to the video file
        self.video_writer1.write(cv_image)

    def listener_callback2(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Initialize video writer if not already initialized
        if self.video_writer2 is None:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer2 = cv2.VideoWriter(
                self.video_file2, fourcc, self.frame_rate2,
                (self.frame_width2, self.frame_height2)
            )

        # Write the frame to the video file
        self.video_writer2.write(cv_image)

    def destroy_node(self):
        # Release the video writer objects
        if self.video_writer1 is not None:
            self.video_writer1.release()
        if self.video_writer2 is not None:
            self.video_writer2.release()
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
