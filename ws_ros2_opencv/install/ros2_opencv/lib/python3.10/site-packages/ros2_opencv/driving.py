import rclpy
from rclpy.node import Node
import math
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import adafruit_motor.servo
import time

class DrivingNode(Node):

    def __init__(self):
        super().__init__('driver_node')
        # Initialization of the servo motor
        self.pca = self.servo_motor_initialization()
        self.get_logger().info('Servo motor initialized.')

        # Timer to wait 10 seconds before starting the car
        self.start_timer = self.create_timer(10.0, self.start_motor)

    def servo_motor_initialization(self):
        # Initialize the I2C bus and PCA9685
        i2c_bus = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c_bus)
        pca.frequency = 100
        return pca

    def motor_speed(self, pca, percent):
        # Converts a -1 to 1 value to a 16-bit duty cycle
        speed = ((percent) * 3277) + 65535 * 0.15
        pca.channels[15].duty_cycle = math.floor(speed)
        self.get_logger().info(f'Motor speed set to {speed / 65535:.2f}')
   
    def start_motor(self):

        # Start the motor after 10 seconds
        self.get_logger().info('Starting the motor after a 10-second delay.')
        self.motor_speed(self.pca, 0.15)  # Start motor with initial speed
        self.get_logger().info('Set speed to initial speed')
        self.destroy_timer(self.start_timer)  # Destroy the start timer

        # Timer to stop the motor after 10 seconds of running
        self.stop_timer = self.create_timer(10.0, self.stop_motor)

    def stop_motor(self):
        # Stop the motor after 10 seconds
        self.motor_speed(self.pca, 0)
        self.get_logger().info('Motor stopped.')
        self.destroy_timer(self.stop_timer)  # Destroy timer to stop calling

def main(args=None):
    rclpy.init(args=args)
    driver_node = DrivingNode()
    rclpy.spin(driver_node)
    
    # Cleanup
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
