import time
import math
import busio
from adafruit_pca9685 import PCA9685
import adafruit_motor.servo as servo
import RPi.GPIO as GPIO
from board import SCL, SDA

# Define servo channels for steering and motor
channel_steering = 14  # steering channel (direction control)
channel_motor = 15     # motor channel (speed control)

# Initialize PCA9685
def Servo_Motor_Initialization():
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
    pca.frequency = 100
    return pca

def Motor_Speed(pca,percent):
   #converts a -1 to 1 value to 16-bit duty cycle
   speed = ((percent) * 3277) + 65535 * 0.15
   pca.channels[15].duty_cycle = math.floor(speed)
   print(speed/65535)

# Initialize servo and motor controls
GPIO.cleanup()  # Reset GPIO state
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup PCA9685 for servo control
pca = Servo_Motor_Initialization()
servo_steering = servo.Servo(pca.channels[channel_steering])

# Define GPIO for ultrasonic sensors
GPIO_TRIGGER_CENTRAL = 18
GPIO_ECHO_CENTRAL = 17
GPIO.setup(GPIO_TRIGGER_CENTRAL, GPIO.OUT)
GPIO.setup(GPIO_ECHO_CENTRAL, GPIO.IN)

GPIO_TRIGGER_RIGHT = 20
GPIO_ECHO_RIGHT = 19
GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)

GPIO_TRIGGER_LEFT = 27
GPIO_ECHO_LEFT = 26
GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)
GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)

# Functions for controlling the motor and steering
def goforward():
    print("Moving forward")
    Motor_Speed(pca,0.15) # Forward motion

def gobackward():
    print("Moving backward")
    Motor_Speed(pca,-0.15)  # Backward motion
    time.sleep(1)
    stopmotors()
    time.sleep(1)
    Motor_Speed(pca,-0.15)  # Backward motion
    time.sleep(1)
    stopmotors()

def straight():
    print("moving straight")
    servo_steering.angle = 90  # straight turn

def turnleft():
    print("Turning left")
    servo_steering.angle = 135  # Left turn

def turnright():
    print("Turning right")
    servo_steering.angle = 45  # Right turn

def stopmotors():
    print("Stopping motors")
    Motor_Speed(pca,0.0)  # Neutral position (stop)
    servo_steering.angle = 90  # Neutral position for steering

# Functions for detecting obstacles
def frontobstacle():
    GPIO.output(GPIO_TRIGGER_CENTRAL, False)
    time.sleep(0.2)
    GPIO.output(GPIO_TRIGGER_CENTRAL, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_CENTRAL, False)
    start = time.time()
    stop = start
    while GPIO.input(GPIO_ECHO_CENTRAL) == 0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_CENTRAL) == 1:
        stop = time.time()
    elapsed = stop - start
    distance = elapsed * 34000 / 2
    print(f"Front Distance: {distance:.1f} cm")
    return distance

def rightobstacle():
    GPIO.output(GPIO_TRIGGER_RIGHT, False)
    time.sleep(0.2)
    GPIO.output(GPIO_TRIGGER_RIGHT, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_RIGHT, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO_RIGHT) == 0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_RIGHT) == 1:
        stop = time.time()
    elapsed = stop - start
    distance = elapsed * 34000 / 2
    print(f"Right Distance: {distance:.1f} cm")
    return distance

def leftobstacle():
    GPIO.output(GPIO_TRIGGER_LEFT, False)
    time.sleep(0.2)
    GPIO.output(GPIO_TRIGGER_LEFT, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_LEFT, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO_LEFT) == 0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_LEFT) == 1:
        stop = time.time()
    elapsed = stop - start
    distance = elapsed * 34000 / 2
    print(f"Left Distance: {distance:.1f} cm")
    return distance

def no_obstacle():
    print("No obstacle detected. Moving forward.")
    stopmotors()   # Stop motors for a brief moment
    time.sleep(1)
    straight()     # Set steering to straight
    time.sleep(1)
    goforward()    # Move forward

# Check front obstacle and turn right if there is an obstacle
def checkanddrivefront():
    stopmotors()
    turnright()
    goforward()

# Check right obstacle and turn left if there is an obstacle
def checkanddriveright():
    stopmotors()
    turnleft()
    goforward()
    time.sleep(2.5)
    turnright()
    goforward()
    time.sleep(1.75)
    straight()
    goforward()

# Check left obstacle and turn right if there is an obstacle
def checkanddriveleft():
    stopmotors()
    turnright()
    goforward()
    time.sleep(2.5)
    turnleft()
    goforward()
    time.sleep(1.75)
    straight()
    goforward()

# Avoid obstacles and drive forward
def obstacleavoiddrive():
    straight()
    start = time.time()

    while time.time() - start < 300:  # 5 minutes = 300 seconds
        if frontobstacle() < 40:
            stopmotors()
            # check left and right
            left = leftobstacle()
            right = rightobstacle()

            if left and right < 50:
                gobackward()
                time.sleep(2)
                stopmotors()
                left = leftobstacle()
                right = rightobstacle()      

            if right > left:
                checkanddriveleft()
            elif left > right:
                checkanddriveright()
            else:
                straight()
                goforward()
        else:
            goforward()  # No obstacle detected; keep moving
    cleargpios()

def cleargpios():
    print("Clearing GPIO")
    GPIO.cleanup()  # Reset GPIO settings
    GPIO.setmode(GPIO.BCM)  # Reinitialize GPIO mode
    # Reconfigure pins for ultrasonic sensors
    GPIO.setup(GPIO_TRIGGER_CENTRAL, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_CENTRAL, GPIO.IN)
    GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)
    GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)
    GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)
    print("All GPIOs cleared and reinitialized")


def main():
    goforward()
    try:
        cleargpios()  # Reset GPIOs and reinitialize
        print("Start driving:")
        obstacleavoiddrive()
    finally:
        stopmotors()
        print ("Motor stopped!")

if __name__ == "__main__":
    main()