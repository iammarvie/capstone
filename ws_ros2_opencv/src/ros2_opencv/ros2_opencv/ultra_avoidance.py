import RPi.GPIO as GPIO  # Import GPIO library
import time  # Import time library

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)  # Programming the GPIO by BCM pin numbers

TRIG = 12  # Define TRIG pin
ECHO = 11  # Define ECHO pin

# Motor control pins
m11 = ##
m12 = ##
m21 = ##
m22 = ##

# Set up the GPIO pins
GPIO.setup(TRIG, GPIO.OUT)  # Initialize TRIG as output
GPIO.setup(ECHO, GPIO.IN)   # Initialize ECHO as input

# Motor pin setup
GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)

time.sleep(5)  # Initial delay

# Define motor control functions
def stop():
    print("stop")
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)

def forward():
    GPIO.output(m11, 1)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    print("Forward")

def back():
    GPIO.output(m11, 0)
    GPIO.output(m12, 1)
    GPIO.output(m21, 0)
    GPIO.output(m22, 1)
    print("back")

def left():
    GPIO.output(m11, 0)
    GPIO.output(m12, 0)
    GPIO.output(m21, 1)
    GPIO.output(m22, 0)
    print("left")

def right():
    GPIO.output(m11, 1)
    GPIO.output(m12, 0)
    GPIO.output(m21, 0)
    GPIO.output(m22, 0)
    print("right")

# Stop the car initially
stop()

# Main loop
count = 0
while True:
    avgDistance = 0
    for i in range(5):  # Take 5 readings to calculate the average distance
        GPIO.output(TRIG, False)  # Set TRIG to LOW
        time.sleep(0.1)

        GPIO.output(TRIG, True)  # Set TRIG to HIGH
        time.sleep(0.00001)  # 10µs pulse
        GPIO.output(TRIG, False)  # Set TRIG to LOW

        # Wait for the echo to start and end
        while GPIO.input(ECHO) == 0:
            pass  # Wait for ECHO to go HIGH (start pulse)

        pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pass  # Wait for ECHO to go LOW (end pulse)

        pulse_end = time.time()

        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Convert time to distance in cm
        distance = round(distance, 2)
        avgDistance += distance

    avgDistance = avgDistance / 5  # Average distance from 5 readings
    print(avgDistance)

    # Obstacle avoidance logic
    if avgDistance < 15:  # If obstacle is within 15 cm
        count += 1
        stop()
        time.sleep(1)
        back()  # Reverse
        time.sleep(1.5)

        if count % 3 == 1:
            right()  # Turn right
        else:
            left()  # Turn left

        time.sleep(1.5)
        stop()
        time.sleep(1)
    else:
        forward()  # Move forward
