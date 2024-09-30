import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # For controlling the car's velocity
from std_msgs.msg import Float32  # Assuming the distance node publishes a Float32 message
import time

class DrivingNode(Node):

    def __init__(self):
        super().__init__('driver_node')
        
        # Publisher to control car's velocity using Twist messages
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        
        self.get_logger().info('Waiting 20 seconds to begin moving.')
        
        # Timer to wait 20 seconds before starting the car
        self.start_timer = self.create_timer(20.0, self.start_motor)
        
        # Initial motor speed
        self.initial_speed = 0.15
        
        # Subscribe to the distance node (distance is in pixels)
        self.subscription = self.create_subscription(Float32, 'stop_sign_detection', self.adjust_speed_based_on_distance, 1)

    def start_motor(self):
        # Start the motor after 20 seconds
        self.get_logger().info('Starting the motor after a 20-second delay.')

        # Publish initial speed using Twist
        twist_msg = Twist()
        twist_msg.linear.x = self.initial_speed  # Forward speed of 0.15
        twist_msg.angular.z = 0.0  # No rotation
        self.publisher_.publish(twist_msg)

        self.get_logger().info('Set speed to initial speed.')
        self.destroy_timer(self.start_timer)  # Destroy the start timer

    def adjust_speed_based_on_distance(self, msg):
        distance_in_pixels = msg.data  # Distance from stop sign in pixels
        
        # Create a Twist message to control the car's speed
        twist_msg = Twist()

        # Logic to adjust speed based on the pixel distance from the stop sign
        # Assuming larger pixel values mean closer to the stop sign
        if distance_in_pixels < 50:  # Far from stop sign
            twist_msg.linear.x = 0.15  # Keep moving at normal speed
        elif 50 <= distance_in_pixels < 150:  # Closer to the stop sign
            twist_msg.linear.x = 0.1  # Slow down
        elif distance_in_pixels >= 150:  # Very close to the stop sign
            twist_msg.linear.x = 0.0  # Stop the car
            self.get_logger().info('Car stopped due to close proximity to the stop sign.')

        # Publish the Twist message to adjust the speed
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    driver_node = DrivingNode()
    rclpy.spin(driver_node)

    # Cleanup
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
