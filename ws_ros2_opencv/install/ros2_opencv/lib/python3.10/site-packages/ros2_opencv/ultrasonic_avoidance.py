import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist  # For controlling car's velocity
from std_msgs.msg import Float32  # Distance in pixels


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Disable GPIO warnings
        GPIO.setwarnings(False)

        # Set GPIO mode to BOARD
        GPIO.setmode(GPIO.BOARD)  # Set GPIO mode to match your working code

        # Set up publisher
        self.publisher_road = self.create_publisher(Float32, 'distance', 1)

        # Define GPIO for central ultrasonic sensor (BOARD mode pin numbers)
        self.GPIO_TRIGGER_CENTRAL = 12
        self.GPIO_ECHO_CENTRAL = 11
        GPIO.setup(self.GPIO_TRIGGER_CENTRAL, GPIO.OUT)  # Trigger > Out
        GPIO.setup(self.GPIO_ECHO_CENTRAL, GPIO.IN)      # Echo < In

        # Timer to measure and publish distance every 0.2 seconds
        self.timer = self.create_timer(0.2, self.measure_and_publish_distance)

        self.get_logger().info("Obstacle avoidance node with continuous measurement started")

    # Continuous measurement and publishing of distance
    def measure_and_publish_distance(self):
        try:
            # Trigger the ultrasonic sensor
            GPIO.output(self.GPIO_TRIGGER_CENTRAL, False)
            time.sleep(0.0001)  # Wait for 100 µs
            GPIO.output(self.GPIO_TRIGGER_CENTRAL, True)
            time.sleep(0.00001)  # Trigger for 10 µs
            GPIO.output(self.GPIO_TRIGGER_CENTRAL, False)
            print ("dawggggggggggggggggggggg")
            # Record the start time
            start_time = time.time()
            while GPIO.input(self.GPIO_ECHO_CENTRAL) == 0:
                start_time = time.time()
            print ("starrrrrrrrrrrrrrrrrrrrrrrrrrrr")
            # Record the end time
            while GPIO.input(self.GPIO_ECHO_CENTRAL) == 1:
                end_time = time.time()

            # Calculate distance based on time difference
            elapsed_time = end_time - start_time
            distance = (elapsed_time * 34300) / 2  # Distance in cm

            # Log and publish distance
            self.get_logger().info(f"Front Distance: {distance:.2f} cm")
            distance_msg = Float32()
            distance_msg.data = distance  # Set distance value in Float32 message
            self.publisher_road.publish(distance_msg)

        except Exception as e:
            self.get_logger().error(f"Error in measuring distance: {e}")

    def destroy_node(self):
        GPIO.cleanup()  # Clean up GPIO pins on shutdown
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
