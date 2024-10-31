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
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback, 1)
        self.publisher_road = self.create_publisher(String, 'lane_info', 1)
        self.publisher_image = self.create_publisher(Image, 'lane_image', 1)

    def listener_callback(self, msg):
        start_time = time.perf_counter()
        try:
            self.get_logger().info('Received an image on image_raw')

            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            processed_image, road_info = self.detect_lane(cv_image)

            processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.publisher_image.publish(processed_image_msg)

            road_info_msg = String()
            road_info_msg.data = road_info
            self.publisher_road.publish(road_info_msg)

            self.min_line_length = 40
            self.max_line_gap = 150
            self.canny_threshold1 = 100
            self.canny_threshold2 = 200

        except CvBridgeError as e:
            self.get_logger().info(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def detect_lane(self, cv_image):

        def preprocess_image(cv_image):
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            equalized_image = cv2.equalizeHist(gray_image)
            blur_image = cv2.GaussianBlur(equalized_image, (5, 5), 0)
            cannied_image = cv2.Canny(blur_image, self.canny_threshold1, self.canny_threshold2)
            return cannied_image
        
        def get_region_of_interest_coordinates(width, height):
            bleft = (int(width * 0), int(height*0.85))
            tleft = (int(width * 0.15), int(height * 0.6))
            tright = (int(width * 0.85), int(height * 0.6))
            bright = (int(width), int (height*0.85))
            return [
                bleft, tleft, tright, bright
            ]
        
        def classify_lines(lines):
            left_lines, right_lines = [], []
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if (x2 - x1) != 0:
                        slope = (y2 - y1) / (x2 - x1)
                        if slope < 0:
                            left_lines.append(line)
                        else:
                            right_lines.append(line)
            return left_lines, right_lines
        
        def calculate_steering_angle(left_line, right_line, width, height):

            if left_line is None or right_line is None:
                return None
            
            lane_center = (
                (left_line[0][2] + right_line[0][2]) // 2,
                (left_line[0][3] + right_line[0][3]) // 2
            )
            image_center = (width // 2, height)
            
            offset = lane_center[0] - image_center[0]
            angle = np.arctan2(offset, height) * (180.0 / np.pi)
            return angle
        
        processed_image = preprocess_image(cv_image)

        height, width = cv_image.shape[:2]
        region_of_interest_coor = get_region_of_interest_coordinates(width, height)
        mask = np.zeros_like(processed_image)
        cv2.fillPoly(mask, [np.array(region_of_interest_coor)], 255)
        cropped_image = cv2.bitwise_and(processed_image, mask)

        lines = cv2.HoughLinesP(cropped_image, 1, np.pi / 180, 20, np.array([]), minLineLength=self.min_line_length, maxLineGap=self.max_line_gap)

        left_lines, right_lines = classify_lines(lines) if lines is not None else ([], [])
        left_line = np.mean(left_lines, axis=0).astype(int) if left_lines else None
        right_line = np.mean(right_lines, axis=0).astype(int) if right_lines else None

        # Draw lines on the image
        if left_line is not None:
            cv_image = cv2.line(cv_image, (left_line[0][0], left_line[0][1]), (left_line[0][2], left_line[0][3]), (0, 255, 0), 3)
        if right_line is not None:
            cv_image = cv2.line(cv_image, (right_line[0][0], right_line[0][1]), (right_line[0][2], right_line[0][3]), (0, 255, 0), 3)

        angle = calculate_steering_angle(left_line, right_line, width, height)
        road_info = f'{angle}' if angle is not None else '0'

        return cv_image, road_info
    
def main(args=None):

    rclpy.init(args=args)

    lane_detection_node = LaneDetectionNode()

    rclpy.spin(lane_detection_node)

    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()