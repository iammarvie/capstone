import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32  # Distance in pixels
import cv2
import numpy as np
import time
import math

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback, 1)
        self.publisher_road = self.create_publisher(Float32, 'lane_info', 1)
        self.publisher_image = self.create_publisher(Image, 'lane_image', 1)
        #self.publisher_image2 = self.create_publisher(Image, 'canny_image', 1)

        self.min_line_length = 20
        self.max_line_gap = 150
        self.canny_threshold1 = 30
        self.canny_threshold2 = 90

        #Stop signal to pause lane detection
        self.pause_lane_detection = False
        self.stop_signal = self.create_subscription(String, 'stop_signal', self.stop_signal_callback, 1)

    def listener_callback(self, msg):
        start_time = time.perf_counter()
        try:
            #self.get_logger().info('Received an image on image_raw')

            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            processed_image, road_info, canny = self.detect_lane(cv_image)

            processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            canny_msg =  self.bridge.cv2_to_imgmsg(canny, 'bgr8')
            self.publisher_image.publish(processed_image_msg)
            #self.publisher_image2.publish(canny_msg)

            road_info_msg = Float32()
            road_info_msg.data = road_info
            self.publisher_road.publish(road_info_msg)

        except CvBridgeError as e:
            self.get_logger().info(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().info(f'Unexpected Error: {e}')
            
    def stop_signal_callback(self, msg):
        if msg.data == 'stop':
            self.pause_lane_detection = True
            self.get_logger().info('Lane detection paused.')
        elif msg.data == 'go':
            self.pause_lane_detection = False
            self.get_logger().info('Lane detection resumed.')

    def detect_lane(self, cv_image):

        # check to start or pause lane detection
        if self.pause_lane_detection:
            return cv_image, 0.0, cv_image

        # Preprocess the image
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # Convert to HSV and apply a color threshold to detect brownish areas
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_brown = np.array([10, 100, 20])  # Adjust these values for the brownish area
        upper_brown = np.array([20, 255, 200])  # Adjust these values for the brownish area
        mask_brown = cv2.inRange(hsv_image, lower_brown, upper_brown)
        highlighted_image = cv2.bitwise_and(image, image, mask=mask_brown)

        # Convert to grayscale and apply histogram equalization
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        equalized_image = cv2.equalizeHist(gray_image)        
        cv2.imwrite('gray.jpg', gray_image)
        #equalized_image = cv2.equalizeHist(gray_image)
        #blur_image = cv2.GaussianBlur(equalized_image, (5, 5), 0)
        cannied_image = cv2.Canny(equalized_image, self.canny_threshold1, self.canny_threshold2)
        # Convert single-channel image to three channels for VideoWriter
        cannied_image_bgr = cv2.cvtColor(cannied_image, cv2.COLOR_GRAY2BGR)
        height, width = cv_image.shape[:2]

        # Define the region of interest
        def get_region_of_interest_coordinates(width, height):
            hpush_b = 0.84
            hpush_t = 0.75
            bleft = (int(width * 0), int(height * hpush_b))  # Bottom-left
            bright = (int(width), int(height * hpush_b))  # Bottom-right
            tleft = (int(width * 0.20), int(height * hpush_t))  # Top-left
            tright = (int(width * 0.80), int(height * hpush_t))  # Top-right
            return [bleft, tleft, tright, bright]

        region_of_interest_coor = get_region_of_interest_coordinates(width, height)
        center_mask = [
            (int(0.5 * width), int(0.5 * height)),  # Top-left
            (int(0.25 * width), int(0.84 * height)),  # Bottom-left
            (int(0.70 * width), int(0.84 * height)),  # Bottom-right
            (int(0.5 * width), int(0.5 * height))   # Top-right
        ]
        mask = np.zeros_like(cannied_image)
        cv2.fillPoly(mask, [np.array(region_of_interest_coor)], 255)
        cropped_image = cv2.bitwise_and(cannied_image, mask)
        cv2.fillPoly(cropped_image, [np.array(center_mask)],0)
        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(
            cropped_image, 1, np.pi / 60, 10, np.array([]),
            minLineLength=self.min_line_length, maxLineGap=self.max_line_gap
        )
        image_center_x = width // 2
        max_y = height  # Bottom of the image
        # Initialize variables for left and right lines
        left_line, right_line = None, None
        lane_center = (image_center_x, max_y)  # Default lane center

        if lines is not None:
            left_lines, right_lines = [], []

            # Classify lines into left and right
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if (x2 - x1) != 0:  # Avoid division by zero
                        slope = (y2 - y1) / (x2 - x1)
                        if slope < -0.5:  # Filter for left lines
                            left_lines.append((x1, y1, x2, y2))
                        elif slope > 0.5:  # Filter for right lines
                            right_lines.append((x1, y1, x2, y2))

            # Compute average left and right lines
            if left_lines:
                left_line = np.mean(left_lines, axis=0).astype(int)
                cv2.line(cv_image, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0, 255, 0), 5)
            if right_lines:
                right_line = np.mean(right_lines, axis=0).astype(int)
                cv2.line(cv_image, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 255, 0), 5)

            # Calculate lane center if both lines exist
            if left_line is not None and right_line is not None:
                lane_center = (
                    (left_line[2] + right_line[2]) // 2,
                    (left_line[3] + right_line[3]) // 2
                )
        if left_line is not None and right_line is not None:
            lane_center = (
                (left_line[2] + right_line[2]) // 2,
                (left_line[3] + right_line[3]) // 2
            )
            offset = lane_center[0] - image_center_x
            angle = np.arctan2(offset, height) * (180.0 / np.pi)
        else:
            if left_line is None and right_line is not None:
                lane_center = (right_line[0], right_line[1])  # Use right line's start point
                offset = lane_center[0] - image_center_x
                angle = np.arctan2(offset, height) * (180.0 / np.pi)
            elif right_line is None and left_line is not None:
                lane_center = (left_line[0], left_line[1])  # Use left line's start point
                offset = lane_center[0] - image_center_x
                angle = np.arctan2(offset, height) * (180.0 / np.pi)
            else:
                lane_center = (image_center_x, max_y)
                offset = 0
                angle = 0.0

        # Calculate offset and angle
        offset = lane_center[0] - image_center_x
        angle = np.arctan2(offset, height) * (180.0 / np.pi)

        # Display angle information and draw lane center line
        cv2.putText(cv_image, f'Angle: {angle:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        if lane_center:
            cv2.circle(cv_image, lane_center, 5, (0, 0, 255), -1)
            cv2.line(cv_image, (image_center_x, max_y), lane_center, (255, 0, 255), 2)

        # Draw region of interest
        cv2.polylines(cv_image, [np.array(region_of_interest_coor)], True, (0, 255, 255), 2)
        cv2.polylines(cv_image, [np.array(center_mask)], True, (0, 255, 255), 2)

        cv2.polylines(cannied_image_bgr, [np.array(region_of_interest_coor)], True, (0, 255, 255), 2)

        # Publish the steering angle as Float32
        road_info = Float32()
        road_info.data = float(angle)
        return cv_image, road_info.data, cannied_image_bgr


    
def main(args=None):

    rclpy.init(args=args)

    lane_detection_node = LaneDetectionNode()

    rclpy.spin(lane_detection_node)

    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
