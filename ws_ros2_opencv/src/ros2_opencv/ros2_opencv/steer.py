import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist  # For controlling the car
import math
from std_msgs.msg import Float32  # Distance in pixels
from board import SCL,SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import time
import adafruit_motor.servo


class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Float32, 'lane_info', self.listener_callback, 1)
        self.twist_publisher = self.create_publisher(Twist, 'cmd_steer', 1) 

    def listener_callback(self, msg):
        lane_info = msg.data
        self.get_logger().info(f'lane_info: {lane_info}')

        twist = Twist()
        twist.angular.z = self.steer(lane_info)

        self.twist_publisher.publish(twist)
        self.get_logger().info(f'Published: {twist.angular.z}')

    def steer(self, lane_info):
        if abs(lane_info) < 2:
            self.get_logger().info('Steering straight')
            return 0.0
        elif lane_info > 2:
            self.get_logger().info('Steering right')
            return -0.35
        else:
            self.get_logger().info('Steering left')
            return 0.35

def main(args=None):

    rclpy.init(args=args)

    steering_node = SteeringNode()

    rclpy.spin(steering_node)

    lane_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()
        
    