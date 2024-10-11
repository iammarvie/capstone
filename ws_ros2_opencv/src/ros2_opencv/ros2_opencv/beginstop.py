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
        self.distance_threshold = 50  # Distance threshold for stopping (adjust accordingly)

    def listener_callback(self, msg):
        # Example data extraction from object detection (adjust to match actual detection data structure)
        # The msg contains object labels and bounding boxes. Assuming a format like: label, bbox_x1, bbox_y1, bbox_x2, bbox_y2
        data = msg.data.split(',')  # Placeholder parsing logic; adjust based on actual message structure
        label = data[0]  # Object label
        bbox_x1, bbox_y1, bbox_x2, bbox_y2 = map(int, data[1:])  # Bounding box coordinates

        # Check if the detected object is a stop sign
        if label == 'stop sign':
            # Calculate the size of the bounding box (can be used to estimate distance)
            bbox_width = bbox_x2 - bbox_x1
            bbox_height = bbox_y2 - bbox_y1

            # Estimate distance based on bounding box size (simplified approach)
            distance_estimate = 1000 / (bbox_width + bbox_height)  # Simplified estimation formula

            # Log the detection
            self.get_logger().info(f'Stop sign detected. Distance estimate: {distance_estimate:.2f}')

            # If the estimated distance is below the threshold, stop the car
            if distance_estimate < self.distance_threshold:
                self.stop_the_car()

    def stop_the_car(self):
        if not self.stop_sign_detected:
            self.get_logger().info('Stopping the car...')
            
            # Publish zero velocity to stop the car
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            
            # Check if the publisher is initialized
            if hasattr(self, 'publisher_') and self.publisher_ is not None:
                self.publisher_.publish(stop_msg)
                self.get_logger().info('Stop command published.')
            else:
                self.get_logger().error('Twist publisher not initialized.')

            self.stop_sign_detected = True  # Prevent multiple stop commands

def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetectionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
