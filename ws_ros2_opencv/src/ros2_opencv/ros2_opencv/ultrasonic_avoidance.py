import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Define GPIO for PWM motor channels
        GPIO.setmode(GPIO.BOARD)
        self.motor_pwm_pin = 14   # PWM pin for motor control (adjust based on your setup)
        GPIO.setup(self.motor_pwm_pin, GPIO.OUT)

        # Set up PWM on motor control pin at 100 Hz
        self.pwm_motor = GPIO.PWM(self.motor_pwm_pin, 100)
        self.pwm_motor.start(0)  # Start with 0% duty cycle (motor off)

        # Define GPIO for central ultrasonic sensor
        self.GPIO_TRIGGER_CENTRAL = 12
        self.GPIO_ECHO_CENTRAL = 11
        GPIO.setup(self.GPIO_TRIGGER_CENTRAL, GPIO.OUT)  # Trigger > Out
        GPIO.setup(self.GPIO_ECHO_CENTRAL, GPIO.IN)      # Echo < In

        # Timer to execute obstacle avoidance
        self.timer = self.create_timer(0.1, self.obstacle_avoid_drive)

        self.get_logger().info("Obstacle avoidance node with PWM motor control started")

    # Functions for driving
    def go_forward(self, speed=50):
        """Move forward with specified speed as a percentage (0-100)."""
        self.pwm_motor.ChangeDutyCycle(speed)
        self.get_logger().info("Moving forward")

    def turn_left(self, speed=50):
        """Turn left with specified speed."""
        self.pwm_motor.ChangeDutyCycle(speed)
        time.sleep(0.8)
        self.pwm_motor.ChangeDutyCycle(0)  # Stop motor

    def turn_right(self, speed=50):
        """Turn right with specified speed."""
        self.pwm_motor.ChangeDutyCycle(speed)
        time.sleep(0.8)
        self.pwm_motor.ChangeDutyCycle(0)  # Stop motor

    def go_backward(self, speed=50):
        """Move backward with specified speed."""
        self.pwm_motor.ChangeDutyCycle(speed)
        self.get_logger().info("Moving backward")

    def stop_motors(self):
        """Stop all motor movement."""
        self.pwm_motor.ChangeDutyCycle(0)
        self.get_logger().info("Motors stopped")

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

    # Check front obstacle and turn right if there is an obstacle
    def check_and_drive_front(self):
        while self.front_obstacle() < 30:
            self.stop_motors()
            self.turn_right()
        self.go_forward()

    # Avoid obstacles and drive forward
    def obstacle_avoid_drive(self):
        self.go_forward()
        if self.front_obstacle() < 30:
            self.stop_motors()
            self.check_and_drive_front()

    def clear_gpios(self):
        self.get_logger().info("Clearing GPIOs and stopping motors")
        self.pwm_motor.stop()
        GPIO.output(self.GPIO_TRIGGER_CENTRAL, False)
        GPIO.cleanup()
        self.get_logger().info("All GPIOs cleared")

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

