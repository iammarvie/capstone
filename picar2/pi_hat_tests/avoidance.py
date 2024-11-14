# External module imports
import RPi.GPIO as GPIO
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

GPIO.setwarnings(False)
GPIO.setmode(GPIO.Board)

def Servo_Motor_Initialization():
   i2c_bus = busio.I2C(SCL,SDA)
   pca = PCA9685(i2c_bus)
   pca.frequency = 100
   return pca

#initialization
pca = Servo_Motor_Initialization()

# Turning servos
channel_num = 14
servo7 = servo.Servo(pca.channels[channel_num])

# Driving 
def Motor_Speed(pca,percent):
   #converts a -1 to 1 value to 16-bit duty cycle
   speed = ((percent) * 3277) + 65535 * 0.15
   pca.channels[15].duty_cycle = math.floor(speed)
   print(speed/65535)


# Define GPIO for ultrasonic central
GPIO_TRIGGER_CENTRAL = 12
GPIO_ECHO_CENTRAL =  11
GPIO.setup(GPIO_TRIGGER_CENTRAL, GPIO.OUT)  # Trigger > Out
GPIO.setup(GPIO_ECHO_CENTRAL, GPIO.IN)      # Echo < In

# Define GPIO for ultrasonic Right
GPIO_TRIGGER_RIGHT = 38
GPIO_ECHO_RIGHT = 35
GPIO.setup(GPIO_TRIGGER_RIGHT, GPIO.OUT)  # Trigger > Out
GPIO.setup(GPIO_ECHO_RIGHT, GPIO.IN)      # Echo < In

# Define GPIO for ultrasonic Left
GPIO_TRIGGER_LEFT = 13
GPIO_ECHO_LEFT = 37
GPIO.setup(GPIO_TRIGGER_LEFT, GPIO.OUT)  # Trigger > Out
GPIO.setup(GPIO_ECHO_LEFT, GPIO.IN)      # Echo < In


# Functions for driving
def goforward():
    Motor_Speed(pca,0.15)


def turnleft():
    servo7.angle = 45


def turnright():
    servo7.angle = 135


def gobackward():
    Motor_Speed(pca,-0.15)


def stopmotors():
    Motor_Speed(pca,0)


# Detect front obstacle
def frontobstacle():

    # Set trigger to False (Low)
    GPIO.output(GPIO_TRIGGER_CENTRAL, False)
    # Allow module to settle
    time.sleep(0.2)
    # Send 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER_CENTRAL, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_CENTRAL, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO_CENTRAL) == 0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_CENTRAL) == 1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop - start
    # Distance pulse travelled in that time is time
    # Multiplied by the speed of sound (cm/s)
    distance = elapsed * 34000 / 2  # distance of both directions so divide by 2
    print (f'Front Distance : %.1f') % distance
    return distance

def rightobstacle():
    # Set trigger to False (Low)
    GPIO.output(GPIO_TRIGGER_RIGHT, False)
    # Allow module to settle
    time.sleep(0.2)
    # Send 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER_RIGHT, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_RIGHT, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO_RIGHT) == 0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_RIGHT) == 1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop - start
    # Distance pulse travelled in that time is time
    # Multiplied by the speed of sound (cm/s)
    distance = elapsed * 34000 / 2  # Distance of both directions so divide by 2
    print (f"Right Distance : %.1f") % distance
    return distance


def leftobstacle():

    # Set trigger to False (Low)
    GPIO.output(GPIO_TRIGGER_LEFT, False)
    # Allow module to settle
    time.sleep(0.2)
    # Send 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER_LEFT, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER_LEFT, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO_LEFT) == 0:
        start = time.time()
    while GPIO.input(GPIO_ECHO_LEFT) == 1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop - start
    # Distance pulse travelled in that time is time
    # Multiplied by the speed of sound (cm/s)
    distance = elapsed * 34000 / 2  # Distance of both directions so divide by 2
    print (f"Left Distance : %.1f") % distance
    return distance


# Check front obstacle and turn right if there is an obstacle
def checkanddrivefront():
    while frontobstacle() < 30:
        stopmotors()
        turnright()
    goforward()


# Check right obstacle and turn left if there is an obstacle
def checkanddriveright():
    while rightobstacle() < 30:
        stopmotors()
        turnleft()
    goforward()


# Check left obstacle and turn right if there is an obstacle
def checkanddriveleft():
    while leftobstacle() < 30:
        stopmotors()
        turnright()
    goforward()


# Avoid obstacles and drive forward
def obstacleavoiddrive():
    goforward()
    start = time.time()
    # Drive 5 minutes
    while start > time.time() - 300:  # 300 = 60 seconds * 5
        if frontobstacle() < 30:
            stopmotors()
            checkanddrivefront()
        elif rightobstacle() < 30:
            stopmotors() 
            checkanddriveright()
        elif leftobstacle() < 30:
            stopmotors()
            checkanddriveleft()
    # Clear GPIOs, it will stop motors       
    cleargpios()


def cleargpios():
    print (f"clearing GPIO")
    GPIO.output(37, False)
    GPIO.output(11, False)
    GPIO.output(13, False)
    GPIO.output(15, False)
    GPIO.output(16, False)
    GPIO.output(33, False)
    GPIO.output(38, False)    
    print (f"All GPIOs CLEARED")


def main():
    # First clear GPIOs
    cleargpios()
    print (f"start driving: ")
    # Start obstacle avoid driving
    obstacleavoiddrive()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
