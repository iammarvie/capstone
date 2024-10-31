import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # For controlling car's velocity


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Set up publisher and subscriber
        self.publisher_road = self.create_publisher(Twist, 'distance', 1)

        # Define GPIO for central ultrasonic sensor
        self.GPIO_TRIGGER_CENTRAL = 12
        self.GPIO_ECHO_CENTRAL = 11
        GPIO.setup(self.GPIO_TRIGGER_CENTRAL, GPIO.OUT)  # Trigger > Out
        GPIO.setup(self.GPIO_ECHO_CENTRAL, GPIO.IN)      # Echo < In

        # Timer to execute obstacle avoidance
        self.timer = self.create_timer(0.1, self.obstacle_avoid_drive)

        self.get_logger().info("Obstacle avoidance node with PWM motor control started")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        try:
            detected = self.front_obstacle()
            self.get_logger().info(f"Detected: {detected}")
            self.publisher_road.publish(detected)
        except Exception as e:
            self.get_logger().info(f"Error: {e}")

    # Functions for driving the car
    # Detect front obstacle
    def front_obstacle(self):
        GPIO.output(self.GPIO_TRIGGER_CENTRAL, False)
        time.sleep(0.2)
        GPIO.output(self.GPIO_TRIGGER_CENTRAL, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER_CENTRAL, False)
        
        start = time.time()
        while GPIO.input(self.GPIO_ECHO_CENTRAL) == 0:
            start = time.time()
        while GPIO.input(self.GPIO_ECHO_CENTRAL) == 1:
            stop = time.time()
        
        elapsed = stop - start
        distance = elapsed * 34000 / 2  # Distance in cm

        self.get_logger().info(f"Front Distance : {distance:.1f} cm")
        return distance

    def destroy_node(self):
        self.clear_gpios()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(obstacle_avoidance_node)
    except KeyboardInterrupt:
        obstacle_avoidance_node.get_logger().info("Shutting down node")
    finally:
        obstacle_avoidance_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

