import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # For controlling car's velocity
from std_msgs.msg import Float32  # Distance in pixels
from std_msgs.msg import String
import math
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import time
from adafruit_motor import servo
import adafruit_motor.servo

class DrivingNode(Node):

    def __init__(self):
        super().__init__('driver_node')

        # Initialize the servo motor using the provided function
        self.pca = self.servo_motor_initialization()
        self.get_logger().info('Servo motor initialized.')

        self.get_logger().info('Waiting 10 seconds to begin moving.')

        # Timer to wait 20 seconds before starting the car
        self.start_timer = self.create_timer(10.0, self.start_motor)
        # Subscribe to Twist commands
        self.twist_subscription = self.create_subscription(Twist, 'cmd_vel', self.stop_callback, 1)
        self.twist_subscription_steer = self.create_subscription(Twist, 'cmd_steer', self.steer_callback, 1)
        self.twist_subscription_ultra = self.create_subscription(Float32, 'distance', self.obstacle_avoid_drive, 1)

        # Initial motor speed
        self.initial_speed = 0.15

        self.servo_steer = self.steer_servo_initialization()
        self.servo_steer.angle = 90
        self.get_logger().info('Servo steer initialized and set to 90(straight).')

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

    def stop_callback(self, msg):
        # stopping the car
        linear_x = msg.linear.x
        if linear_x == 0.0:
            self.motor_speed(0)  # Stop the car
            self.get_logger().info('Car stopped.')

    def steer_servo_initialization(self):
        # set servo to initial direction
        self.servo_steer = servo.Servo(self.pca.channels[14], min_pulse=600, max_pulse=2400)
        return self.servo_steer

    # Steer servo based on angular z value
    def steer_callback(self, msg):
        # convert angle for servo to use
        angular_z = msg.angular.z
        # convert angle so the servo can use it
        servo_angle  = 90 + (angular_z * 90)
        servo_angle = max(0, min(180, servo_angle))
        self.servo_steer.angle = servo_angle
        self.get_logger().info(f'Steering angle set to {servo_angle}.')

    ## ULTRASONIC AVOIDANCE
    def obstacle_avoid_drive(self, msg):
        distance = msg.data
        if distance < 50:
            self.motor_speed(0)
            self.get_logger().info('Obstacle detected. Stopping the car.')    

def main(args=None):
    rclpy.init(args=args)
    driver_node = DrivingNode()
    rclpy.spin(driver_node)

    # Cleanup
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
