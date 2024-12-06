import cv2
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import time

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
pca.frequency = 100
# channel you are using on the pwm board (should be 14 if you are using the pi hat)
channel_num = 0

servo7 = servo.Servo(pca.channels[channel_num])
servo8 = servo.Servo(pca.channels[1])

servo7.angle = 98 #99
print("left and right")
time.sleep(0.3)
servo8.angle = 105 #100
print("up and down")
time.sleep(0.3)

cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L) 

# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

# take frame
ret, frame = cap.read()

if not ret:
    print("Failed to grab frame")
else:
    # write frame to file
    frame = frame = cv2.flip(frame,-1)
    cv2.imwrite('image.jpg', frame)

# release camera
cap.release()
