
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
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        kernel = np.ones((3, 3), np.float32) / 9
        denoised_image = cv2.filter2D(cv_image, -1, kernel)
        height, width = cv_image.shape[:2]
        gray_image = cv2.cvtColor(denoised_image, cv2.COLOR_BGR2GRAY)
        cannied_image = cv2.Canny(gray_image, 100, 200)

        bleft = (int(width * 0), int(height*0.85))  # Bottom-left
        bright = (int(width), int(height*0.85))# Bottom-right]
        tleft = (int(width * 0.25), int(height * 0.6))  # Top-left
        tright = (int(width * 0.75), int(height * 0.6))  # Top-right

        # Define region of interest
        region_of_interest_coor = [ bleft, tleft, tright,bright ]
        
        mask = np.zeros_like(cannied_image)
        cv2.fillPoly(mask, [np.array(region_of_interest_coor)], 255)
        cropped_image = cv2.bitwise_and(cannied_image, mask)

        # Detect lines using Hough Line Transform
        lines = cv2.HoughLinesP(cropped_image, 1, np.pi / 180, 20, minLineLength=40, maxLineGap=150)

        # Define the four points of the original image
        offset = 30
        src = np.float32(region_of_interest_coor)
        # Define the four points of the desired output
        dst = np.float32([(0, height), (0, 0), (width, 0), (width, height)])
                # Perspective points to be warped
        source_points = np.float32([
            tleft,     # Top-left point
            tright,     # Top-right point
            bleft,               # Bottom-left point
            bright])              # Bottom-right point
        # Window to be shown
        destination_points = np.float32([
            [offset, 0],                             # Top-left point
            [width-2*offset, 0],                     # Top-right point
            [offset, height],                        # Bottom-left point
            [width-2*offset, height]])               # Bottom-right point
                # Matrix to warp the image for skyview window
        matrix = cv2.getPerspectiveTransform(source_points, destination_points)
        # Final warping perspective
        skyview = cv2.warpPerspective(cv_image, matrix, (width, height))
        # Apply Hough Line Transform to detect lines
        lines = cv2.HoughLinesP(cropped_image, 1, np.pi / 180, 20, np.array([]), minLineLength=40, maxLineGap=150)

        # Check if lines were detected before processing them
        left_line = []
        right_line = []

        if lines is not None:  # Ensure lines are detected
            for line in lines:
                for x1, y1, x2, y2 in line:
                    slope = (y2 - y1) / (x2 - x1)
                    if slope < 0:
                        left_line.append(line)
                    else:
                        right_line.append(line)

            if len(left_line) > 0:
                left_line = np.mean(left_line, axis=0, dtype=np.int32)

            if len(right_line) > 0:
                right_line = np.mean(right_line, axis=0, dtype=np.int32)

            # Draw the lines on the image
            cv2.line(cv_image, (left_line[0][0], left_line[0][1]), (left_line[0][2], left_line[0][3]), (0, 255, 0), 5)
            cv2.line(cv_image, (right_line[0][0], right_line[0][1]), (right_line[0][2], right_line[0][3]), (0, 255, 0), 5)

           # Calculate the lane center (midpoint of left and right lines at bottom)
            lane_center = (
                (left_line[0][2] + right_line[0][2]) // 2,
                (left_line[0][3] + right_line[0][3]) // 2 )

            # Calculate the image center
            image_center = (width // 2, height)

            # Calculate the horizontal offset between lane center and image center
            offset = lane_center[0] - image_center[0]

            # Predict the turn angle (in degrees) using arctangent
            angle = np.arctan2(offset, height) * (180.0 / np.pi)
                # Print turn prediction based on angle threshold
            if abs(angle) < 5:
                print("Keep straight")
            elif angle > 5:
                print(f"Turn right by {angle:.2f} degrees")
            else:
                print(f"Turn left by {abs(angle):.2f} degrees")
            print(f'Offset: {offset}, Angle: {angle:.2f} degrees')
            # Display the predicted turn on the image
            cv2.circle(cv_image, lane_center, 5, (0, 0, 255), -1)  # Lane center
            cv2.line(cv_image, image_center, lane_center, (255, 0, 255), 2)  # Offset line
        # Create road information string
            road_info = f'{angle}'
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
