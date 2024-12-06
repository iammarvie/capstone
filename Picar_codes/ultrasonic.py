import RPi.GPIO as GPIO
import argparse
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Argument parser
parser = argparse.ArgumentParser(description="Distance finder")
parser.add_argument('--tim', type=float, default=30.0, action='store', help='Duration of measurement (in seconds)')
parser.add_argument('--debug', action='store_true', help='Enable debug mode')
parser.add_argument('--delay', type=float, default=0.1, action='store', help='Delay between measurements (in seconds)')
args = parser.parse_args()

# GPIO pins for ultrasonic sensor
TRIG = 18
ECHO = 17

GPIO.setup(TRIG, GPIO.OUT)
GPIO.output(TRIG, False)
GPIO.setup(ECHO, GPIO.IN)

# Delay before starting the measurement
time.sleep(args.delay)

# Set up DEBUG mode
DEBUG = args.debug

# Start measuring distance
start_time = time.time()

while time.time() - start_time < args.tim:
    # Send a pulse to trigger the sensor
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Measure the time it takes for the pulse to return
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate the distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 34300 / 2  # Distance in cm

    # Print the distance
    print(f'Distance: {distance:.2f} cm')

    # Debug mode printing additional information
    if DEBUG:
        print(f'Taken in {pulse_duration:.3f} seconds')

    # Wait for the next measurement
    time.sleep(args.delay)

# Clean up GPIO pins
GPIO.cleanup()
