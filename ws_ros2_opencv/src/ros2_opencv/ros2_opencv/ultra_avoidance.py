import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

class ObjectAvoidanceNode(Node):

    def __init__(self):
        super().__init__('object_avoidance_node')

        # Set up GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Define pins
        self.TRIG = 12
        self.ECHO = 11
        self.m11 = 20  # Replace with your actual pin number
        self.m12 = 21  # Replace with your actual pin number
        self.m21 = 22  # Replace with your actual pin number
        self.m22 = 23  # Replace with your actual pin number

        # Initialize GPIO pins
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        GPIO.setup(self.m11, GPIO.OUT)
        GPIO.setup(self.m12, GPIO.OUT)
        GPIO.setup(self.m21, GPIO.OUT)
        GPIO.setup(self.m22, GPIO.OUT)

        # Stop the car initially
        self.stop()

        # Variable to keep track of avoidance logic
        self.count = 0

        # Timer to run the obstacle avoidance logic every 0.5 seconds
        self.timer = self.create_timer(0.5, self.obstacle_avoidance_callback)
        self.get_logger().info('Object avoidance node has been initialized')

    def obstacle_avoidance_callback(self):
        avg_distance = self.calculate_average_distance()

        self.get_logger().info(f'Average Distance: {avg_distance} cm')

        # Obstacle avoidance logic
        if avg_distance < 15:
            self.count += 1
            self.stop()
            time.sleep(1)
            self.back()
            time.sleep(1.5)

            if self.count % 3 == 1:
                self.right()
            else:
                self.left()

            time.sleep(1.5)
            self.stop()
            time.sleep(1)
        else:
            self.forward()

    def calculate_average_distance(self):
        avg_distance = 0
        for _ in range(5):  # Take 5 readings
            GPIO.output(self.TRIG, False)
            time.sleep(0.1)

            GPIO.output(self.TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.TRIG, False)

            while GPIO.input(self.ECHO) == 0:
                pass  # Wait for ECHO to go HIGH

            pulse_start = time.time()

            while GPIO.input(self.ECHO) == 1:
                pass  # Wait for ECHO to go LOW

            pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            avg_distance += round(distance, 2)

        return avg_distance / 5  # Average of 5 readings

    # Motor control functions
    def stop(self):
        self.get_logger().info('Stopping')
        GPIO.output(self.m11, 0)
        GPIO.output(self.m12, 0)
        GPIO.output(self.m21, 0)
        GPIO.output(self.m22, 0)

    def forward(self):
        self.get_logger().info('Moving Forward')
        GPIO.output(self.m11, 1)
        GPIO.output(self.m12, 0)
        GPIO.output(self.m21, 1)
        GPIO.output(self.m22, 0)

    def back(self):
        self.get_logger().info('Reversing')
        GPIO.output(self.m11, 0)
        GPIO.output(self.m12, 1)
        GPIO.output(self.m21, 0)
        GPIO.output(self.m22, 1)

    def left(self):
        self.get_logger().info('Turning Left')
        GPIO.output(self.m11, 0)
        GPIO.output(self.m12, 0)
        GPIO.output(self.m21, 1)
        GPIO.output(self.m22, 0)

    def right(self):
        self.get_logger().info('Turning Right')
        GPIO.output(self.m11, 1)
        GPIO.output(self.m12, 0)
        GPIO.output(self.m21, 0)
        GPIO.output(self.m22, 0)

    def destroy_node(self):
        # Cleanup GPIO on node shutdown
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
