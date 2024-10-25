import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()  # Initialize CvBridge
        self.subscription = self.create_subscription(
            Image,
            'image_raw',  # Topic to subscribe to
            self.listener_callback, 1)
        self.publisher_road = self.create_publisher(String, 'lane_info', 1)
        self.publisher_image = self.create_publisher(Image, 'lane_image', 1)

    def listener_callback(self, msg):
        start_time = time.perf_counter()
        try:
            self.get_logger().info('Received an image on image_raw')

            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Perform lane detection
            processed_image, road_info = self.detect_lane(cv_image)

            # Publish the processed image
            processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.publisher_image.publish(processed_image_msg)

            # Publish the road information
            road_info_msg = String()
            road_info_msg.data = road_info
            self.publisher_road.publish(road_info_msg)

        except CvBridgeError as e:
            self.get_logger().info(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def detect_lane(self, cv_image):
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        kernel = np.ones((3, 3), np.float32) / 9
        denoised_image = cv2.filter2D(cv_image, -1, kernel)
        height, width = cv_image.shape[:2]
        gray_image = cv2.cvtColor(denoised_image, cv2.COLOR_BGR2GRAY)
        cannied_image = cv2.Canny(gray_image, 100, 200)

        # Define region of interest
        region_of_interest_coor = [
            (int(width * 0.1), height),  # Bottom-left
            (int(width * 0.4), int(height * 0.7)),  # Top-left
            (int(width * 0.6), int(height * 0.7)),  # Top-right
            (int(width * 0.96), height)  # Bottom-right
        ]
        mask = np.zeros_like(cannied_image)
        cv2.fillPoly(mask, [np.array(region_of_interest_coor)], 255)
        cropped_image = cv2.bitwise_and(cannied_image, mask)

        # Detect lines using Hough Line Transform
        lines = cv2.HoughLinesP(
            cropped_image, 1, np.pi / 180, 20,
            minLineLength=40, maxLineGap=150)

        left_line, right_line = [], []
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    slope = (y2 - y1) / (x2 - x1)
                    if slope < 0:
                        left_line.append(line)
                    else:
                        right_line.append(line)

            # Calculate the average of left and right lines
            left_line = np.mean(left_line, axis=0, dtype=np.int32) if len(left_line) > 0 else None
            right_line = np.mean(right_line, axis=0, dtype=np.int32) if len(right_line) > 0 else None

            # Draw lines on the image if detected
            if left_line is not None:
                cv2.line(cv_image, (left_line[0], left_line[1]),
                         (left_line[2], left_line[3]), (0, 255, 0), 5)
            if right_line is not None:
                cv2.line(cv_image, (right_line[0], right_line[1]),
                         (right_line[2], right_line[3]), (0, 255, 0), 5)

        # Create road information string
            road_info = f'Left Line: {left_line}, Right Line: {right_line}'
        else:
            road_info = "No lanes detected"
        return cv_image, road_info


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
