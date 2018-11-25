#servos.py
import time
from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

import serial
import math
from roboclaw import Roboclaw

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
br_motor = servo.Servo(pca.channels[0],actuation_range=119,min_pulse=700,max_pulse=2300)
fr_motor = servo.Servo(pca.channels[1])
fl_motor = servo.Servo(pca.channels[2])
bl_motor = servo.Servo(pca.channels[3])

rc = Roboclaw("/dev/ttyS0",115200)
i = rc.Open()
print("open status = ["+str(i)+"]")

steer = 0
speed = 0
left_limit = -40
right_limit = 40

def turn_left(steer):
    fl_motor.angle = 90 + steer
    fr_motor.angle = 90 + steer
    bl_motor.angle = 90 - steer
    br_motor.angle = 90 - steer

def turn_right(steer):
    fl_motor.angle = 90 - steer
    fr_motor.angle = 90 - steer
    bl_motor.angle = 90 + steer
    br_motor.angle = 90 + steer
    
def straighten_up():
    fl_motor.angle = 90
    time.sleep(.1)
    fr_motor.angle = 90
    time.sleep(.1)
    bl_motor.angle = 90
    time.sleep(.1)
    br_motor.angle = 90
    time.sleep(.1)

 
while True:
    key = input("...")
    if key != "":               #repeat?
        cmd = key
    print("cmd="+cmd)
    
    if cmd == 'j':
        print('left')
        if steer > left_limit:
            steer -= 1
            turn_left(steer)
 
    elif cmd == 'J':
        print('LEFT')
        if steer > left_limit - 3:
            steer -= 3
            turn_left(steer)
 
    elif cmd == 'k':
        print('right')
        if steer < right_limit:
            steer += 1
            turn_right(steer)
    
    elif cmd == 'K':
        print('RIGHT')
        if steer < right_limit + 3:
            steer += 3
            turn_right(steer)
    
    elif cmd == ' ':
        print('space')
        steer = 0
        fl_motor.angle = 90
        fr_motor.angle = 90
        bl_motor.angle = 90
        br_motor.angle = 90
        rc.ForwardM1(0x80, 0);
        rc.ForwardM2(0x80, 0);
        rc.ForwardM1(0x81, 0);
        rc.ForwardM2(0x81, 0);
        rc.ForwardM1(0x82, 0);
        rc.ForwardM2(0x82, 0);

    elif cmd == 'f':
        rc.ForwardM1(0x80, 100);
        rc.ForwardM2(0x80, 100);
        rc.ForwardM1(0x81, 100);
        rc.ForwardM2(0x81, 100);
        rc.ForwardM1(0x82, 100);
        rc.ForwardM2(0x82, 100);


    elif cmd == 'q' or cmd == 'Q':
        break
    

pca.deinit()
