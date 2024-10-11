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

        # Start the motor at the initial speed using the motor control function
        self.motor_speed(self.initial_speed)

        self.get_logger().info('Set speed to initial speed.')
        self.destroy_timer(self.start_timer)  # Destroy the start timer

    def twist_callback(self, msg):
        linear_x = msg.linear.x
        if linear_x == 0:
            self.motor_speed(0)  # Stop the car
            self.get_logger().info('Car stopped.')
        else:
            self.motor_speed(linear_x)  # Adjust speed based on Twist command

def main(args=None):
    rclpy.init(args=args)
    driver_node = DrivingNode()
    rclpy.spin(driver_node)

    # Cleanup
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
