import math
from board import SCL,SDA
import busio
from adafruit_pca9685 import PCA9685
import time
import adafruit_motor.servo
import RPi.GPIO as IO
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import sys
import argparse
import smbus
from time import sleep
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

IO.setwarnings(False)
IO.setmode(IO.BCM)

pin_num = 26
IO.setup(pin_num,IO.IN,IO.PUD_UP)

start_time = time.time()
timeLimit = 5
samplePeriod = .5
speeds = []
n = 0

def read():
    gear_rotations = 0
    time_start = time.time()
    time_end = time.time()
    ready_to_go = False
    while gear_rotations < 5:
        curr_pin_val = IO.input(pin_num)
        if curr_pin_val == 1 and ready_to_go == False:
            ready_to_go = True
            gear_rotations += 1
        if curr_pin_val == 0 and ready_to_go == True:
            ready_to_go = False
        if time.time() > time_start + 0.5:
            return 0
        #print(gear_rotations)
    #print('hi!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
    time_end = time.time()
    #speed = (math.pi*.111 / (time_end - time_start)) / 1000
    speed = (2*math.pi*.111) / (time_end - time_start)
    time_start = time.time()
    return speed


def Servo_Motor_Initialization():
   i2c_bus = busio.I2C(SCL,SDA)
   pca = PCA9685(i2c_bus)
   pca.frequency = 100
   return pca

def Motor_Start(pca):
   x = input("Press and hold EZ button. Once the LED turns red, immediately relase the button. After the LED blink red once, press 'ENTER'on keyboard.")
   Motor_Speed(pca, 1)
   time.sleep(2)
   y = input("If the LED just blinked TWICE, then press the 'ENTER'on keyboard.")
   Motor_Speed(pca, -1)
   time.sleep(2)
   z = input("Now the LED should be in solid green, indicating the initialization is complete. Press 'ENTER' on keyboard to proceed")
   

def Motor_Speed(pca,percent):
   #converts a -1 to 1 value to 16-bit duty cycle
   speed = ((percent) * 3277) + 65535 * 0.15
   pca.channels[15].duty_cycle = math.floor(speed)
   print(speed/65535)

#initialization
pca = Servo_Motor_Initialization()
Motor_Start(pca)
Motor_Speed(pca, 0.15)   

while time.time()-start_time < timeLimit:
  if time.time()-start_time > n*samplePeriod:
    speeds.append(read())
    n = n+1

Motor_Speed(pca, 0) 
GPIO.cleanup(0)

x = input("Press 1 to create speed vs. time figure")
title = input('Enter filename for the figure (w/ .png)')

if x == 1:
  plt.clf()
  time = np.linspace(0, samplePeriod, timeLimit)
  plt.plot(time, speeds)
  plt.grid(True)
  plt.xlabel("Time (s)")
  plt.ylabel("Speed")
  plt.savefig(title)
