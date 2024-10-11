import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # For controlling car's velocity
from std_msgs.msg import Float32  # Distance in pixels
import math
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import time

class DrivingNode(Node):

    def __init__(self):
        super().__init__('driver_node')

        # Initialize the servo motor using the provided function
        self.pca = self.servo_motor_initialization()
        self.get_logger().info('Servo motor initialized.')

        self.get_logger().info('Waiting 20 seconds to begin moving.')

        # Timer to wait 20 seconds before starting the car
        self.start_timer = self.create_timer(20.0, self.start_motor)

        # Subscribe to the distance node (distance is in pixels)
        self.subscription = self.create_subscription(Float32, 'stop_sign_detection', self.adjust_speed_based_on_distance, 1)

        # Subscribe to Twist commands
        self.twist_subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 1)

        # Initial motor speed
        self.initial_speed = 0.15

        begin_timer = time.time()

    def servo_motor_initialization(self):
        # Initialize the I2C bus and PCA9685
        i2c_bus = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c_bus)
        pca.frequency = 100
        return pca

    def motor_speed(self, percent):
        # Converts a -1 to 1 value to a 16-bit duty cycle
        speed = ((percent) * 3277) + 65535 * 0.15
        self.pca.channels[15].duty_cycle = math.floor(speed)
        self.get_logger().info(f'Motor speed set to {speed / 65535:.2f}')

    def start_motor(self):
        # Start the motor after 20 seconds
        self.get_logger().info('Starting the motor after a 20-second delay.')

        # Start the motor at the initial speed using the motor control function
        self.motor_speed(self.initial_speed)

        self.get_logger().info('Set speed to initial speed.')
        self.destroy_timer(self.start_timer)  # Destroy the start timer

    def adjust_speed_based_on_distance(self, msg):
        distance_in_pixels = msg.data  # Distance from stop sign in pixels
        
        # Logic to adjust speed based on the pixel distance from the stop sign
        if distance_in_pixels < 50:  # Far from stop sign
            speed = 0.15  # Keep moving at normal speed
        elif 50 <= distance_in_pixels < 150:  # Closer to the stop sign
            speed = 0.1  # Slow down
        elif distance_in_pixels >= 150:  # Very close to the stop sign
            speed = 0.0  # Stop the car
            self.get_logger().info('Car stopped due to close proximity to the stop sign.')

        # Set motor speed using the motor control function
        self.motor_speed(speed)

    def twist_callback(self, msg):
        stop_timer = time.time()
        # Extract the linear.x from the Twist message (forward velocity)
        linear_speed = msg.linear.x

        # Translate this linear speed to motor speed
        self.motor_speed(linear_speed)

        if stop_timer - begin_timer > 50:
            self.motor_speed(0)

        self.get_logger().info(f'Twist command received: linear.x = {linear_speed:.2f}')

def main(args=None):
    rclpy.init(args=args)
    driver_node = DrivingNode()
    rclpy.spin(driver_node)

    # Cleanup
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
