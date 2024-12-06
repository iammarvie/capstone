import time

from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 100
# channel you are using on the pwm board (should be 14 if you are using the pi hat)
channel_num = 0

servo7 = servo.Servo(pca.channels[channel_num])
servo8 = servo.Servo(pca.channels[1])

# We sleep in the loops to give the servo time to move into position.
'''

for i in range(180):
    servo7.angle = i
    print ("first angle",servo7.angle)
    time.sleep(0.03)
for i in range(180):
    servo7.angle = 180 - i
    print ("second angle",servo7.angle)
    time.sleep(0.03)

# You can also specify the movement fractionally.
fraction = 0.0
while fraction < 1.0:
    servo7.fraction = fraction
    fraction += 0.01
    time.sleep(0.03)

    servo7.angle = 90
    time.sleep(3)
    print ("straight")
'''
servo7.angle = 45
print("left")
time.sleep(3)
servo7.angle = 135
print ("right")
time.sleep(3)
servo7.angle = 90
time.sleep(0.3)
print ("straight")
