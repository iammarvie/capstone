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
        self.publisher_image2 = self.create_publisher(Image, 'canny_image', 1)

        self.min_line_length = 40
        self.max_line_gap = 150
        self.canny_threshold1 = 50
        self.canny_threshold2 = 120

    def listener_callback(self, msg):
        start_time = time.perf_counter()
        try:
            #self.get_logger().info('Received an image on image_raw')

            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            processed_image, road_info, canny = self.detect_lane(cv_image)

            processed_image_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            canny_msg =  self.bridge.cv2_to_imgmsg(canny, 'bgr8')
            self.publisher_image.publish(processed_image_msg)
            self.publisher_image2.publish(canny_msg)

            road_info_msg = Float32()
            road_info_msg.data = road_info
            self.publisher_road.publish(road_info_msg)

        except CvBridgeError as e:
            self.get_logger().info(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().info(f'Error: {e}')

    def detect_lane(self, cv_image):

        # Preprocess the image
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        kernel = np.ones((3, 3), np.float32) / 9
        #denoised_image = cv2.filter2D(image, -1, kernel)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imwrite('gray.jpg', gray_image)
        #equalized_image = cv2.equalizeHist(gray_image)
        #blur_image = cv2.GaussianBlur(equalized_image, (5, 5), 0)
        cannied_image = cv2.Canny(gray_image, self.canny_threshold1, self.canny_threshold2)
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
            (int(0.20 * width), int(0.84 * height)),  # Bottom-left
            (int(0.80 * width), int(0.84 * height)),  # Bottom-right
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

        # Classify lines into left and right
        left_lines_x, left_lines_y = [], []
        right_lines_x, right_lines_y = [], []
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if (x2 - x1) != 0:
                        slope = (y2 - y1) / (x2 - x1)
                        if math.fabs(slope) < 0.5:  # Filter out lines that are too horizontal
                            continue
                        if slope < 0:
                            left_lines_x.extend([x1, x2])
                            left_lines_y.extend([y1, y2])
                        else:
                            right_lines_x.extend([x1, x2])
                            right_lines_y.extend([y1, y2])

        # Calculate the steering angle and draw lines
        def calculate_steering_angle_and_draw(left_x, left_y, right_x, right_y, width, height, image):
            if not left_x or not right_x or not left_y or not right_y:
                return 0.0  # Return 0 if either line data is missing

            # Fit lines to the left and right points
            poly_left = np.poly1d(np.polyfit(left_y, left_x, deg=1))
            poly_right = np.poly1d(np.polyfit(right_y, right_x, deg=1))

            # Calculate the x-coordinates of the lines at min_y and max_y
            min_y = int(height * 0.6)  # Just below the horizon
            max_y = height  # Bottom of the image

            left_x_start = int(poly_left(max_y))
            left_x_end = int(poly_left(min_y))
            right_x_start = int(poly_right(max_y))
            right_x_end = int(poly_right(min_y))

            # Draw the left and right lines on the image
            cv2.line(image, (left_x_start, max_y), (left_x_end, min_y), (0, 255, 0), 5)
            cv2.line(image, (right_x_start, max_y), (right_x_end, min_y), (0, 255, 0), 5)

            # Calculate lane center
            lane_center_x = (left_x_end + right_x_end) // 2
            lane_center = (lane_center_x, min_y)

            # Calculate image center and offset
            image_center_x = width // 2
            offset = lane_center_x - image_center_x

            # Calculate the steering angle
            angle = np.arctan2(offset, height) * (180.0 / np.pi)
            return float(angle)

        # Calculate the angle and draw lines
        angle = calculate_steering_angle_and_draw(left_lines_x, left_lines_y, right_lines_x, right_lines_y, width, height, cv_image)

        # Display angle information on the image
        cv2.putText(cv_image, f'Angle: {angle:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

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
