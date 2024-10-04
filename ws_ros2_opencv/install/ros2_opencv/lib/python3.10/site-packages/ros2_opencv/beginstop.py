# This node will take info from the object detection node and sees when a stop sign has been detected as the car is moving. Using the data from the object detection node, the car will stop when a stop sign is detected.
#using the bodundary boxes, it will detect how far the stop sign is from the car and stop the car when it is close enough.
#The distance threshold is set to 50 pixels, but this can be adjusted based on the actual detection data and the camera's field of view.
#The stop_the_car method publishes a zero velocity Twist message to stop the car. It also sets a flag to prevent multiple stop commands from being sent.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # For controlling the car
from std_msgs.msg import String  # Example for object detection message, adjust accordingly
from sensor_msgs.msg import Image  # If image data is needed
import time

class StopSignDetectionNode(Node):
    def __init__(self):
        super().__init__('stop_sign_detection')

        # Subscribe to the object detection node's output
        self.subscription = self.create_subscription(String, 'object_detection', self.listener_callback, 1)

        # Publisher to control car's movement (cmd_vel)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # Parameters for stopping behavior
        self.stop_sign_detected = False
        self.distance_threshold = 50  # Distance threshold in pixels for stopping (adjust accordingly)

    def listener_callback(self, msg):
        # Example data extraction from object detection (adjust to match actual detection data structure)
        # The msg contains object labels and bounding boxes. Assuming a format like: label, bbox_x1, bbox_y1, bbox_x2, bbox_y2
        data = msg.data.split(',')  # Placeholder parsing logic; adjust based on actual message structure
        label = data[0]  # Object label
        bbox_x1, bbox_y1, bbox_x2, bbox_y2 = map(int, data[1:])  # Bounding box coordinates

        # Check if the detected object is a stop sign
        if label == 'stop_sign':
            # Calculate the size of the bounding box (can be used to estimate distance)
            bbox_width = bbox_x2 - bbox_x1
            bbox_height = bbox_y2 - bbox_y1

            # Estimate distance based on bounding box size (simplified approach)
            distance_estimate = 1000 / (bbox_width + bbox_height)  # Simplified estimation formula

            # Log the detection
            self.get_logger().info(f'Stop sign detected. Distance estimate: {distance_estimate:.2f}')

            # If the bounding box indicates that the stop sign is close enough, stop the car
            if bbox_width > self.distance_threshold or bbox_height > self.distance_threshold:
                self.stop_the_car()

    def stop_the_car(self):
        if not self.stop_sign_detected:
            self.get_logger().info('Stopping the car...')
            # Publish zero velocity to stop the car
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher_.publish(stop_msg)
            self.stop_sign_detected = True  # Prevent multiple stop commands

def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetectionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
