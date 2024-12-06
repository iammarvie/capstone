import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define pin numbers for each sensor
sensors = {
    "central": {"trigger": 18, "echo": 17},
    "right": {"trigger": 20, "echo": 19},
    "left": {"trigger": 27, "echo": 26}
}

# Set up each sensor's pins
for sensor in sensors.values():
    GPIO.setup(sensor["trigger"], GPIO.OUT)
    GPIO.setup(sensor["echo"], GPIO.IN)
    GPIO.output(sensor["trigger"], False)

# Function to measure distance from a single sensor
def measure_distance(trigger_pin, echo_pin):
    # Send a 10µs pulse to trigger the measurement
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(trigger_pin, False)
    
    # Wait for the echo pin to go high
    start_time = time.time()
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
    
    # Wait for the echo pin to go low
    stop_time = time.time()
    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()
    
    # Calculate the distance
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2  # Speed of sound in cm/s
    return distance

try:
    # Start time for 10 seconds duration
    start_time = time.time()
    
    while time.time() - start_time < 10:  # Run for 10 seconds
        distances = {}
        for sensor_name, pins in sensors.items():
            distance = measure_distance(pins["trigger"], pins["echo"])
            distances[sensor_name] = distance
            print(f"{sensor_name.capitalize()} Distance: {distance:.2f} cm")
        
        # Pause between readings
        time.sleep(1)

except KeyboardInterrupt:
    print(f"Measurement stopped by User")

finally:
    GPIO.cleanup()
