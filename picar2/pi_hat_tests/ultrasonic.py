import RPi.GPIO as GPIO
import sys
import argparse
import time
from time import  sleep
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

parser = argparse.ArgumentParser(description = "distance finder")
parser.add_argument ('--tim', type = float, default = 30.0,  action = 'store')
parser.add_argument ('--debug',  action = 'store_true', help = 'debug')
parser.add_argument ('--delay', type = float, default = 0.1, action = 'store')
args = parser.parse_args()

TRIG = 12
ECHO = 11

GPIO.setup(TRIG, GPIO.OUT)
GPIO.output(TRIG, False)
GPIO.setup(ECHO, GPIO.IN)

time.sleep(args.delay)
time_start = time.time()
time_cur=time_start
sleep_time=time_cur

GPIO.output(TRIG, True)
time.sleep(0.00001)
GPIO.output(TRIG,False)

DEBUG = False

while time_cur-time_start<args.tim:
   if sleep_time>=args.delay:
      GPIO.output(TRIG,True)
      time.sleep(0.00001)
      GPIO.output(TRIG, False)
      while GPIO.input(ECHO) == 0:
         start_time = time.time()
      while GPIO.input(ECHO) == 1:
         end_time = time.time()
      time_end = time.time()
      total_distance = (end_time - start_time)* 34300
      print (f'Distance: {total_distance/2:.2f} cm') 
   time_cur=time.time()
   sleep_time=time_cur-end_time

   if '--debug' in sys.argv:
      DEBUG = True
      print (f'Distance: {total_distance/2:.2f} cm')
      print (f'Taken in {time_end - time_start:.3f} seconds')

GPIO.cleanup()
