#servos.py
import time
from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685

from adafruit_motor import servo

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
br_motor = servo.Servo(pca.channels[0],actuation_range=119,min_pulse=700,max_pulse=2300)
fr_motor = servo.Servo(pca.channels[1])
fl_motor = servo.Servo(pca.channels[2])
bl_motor = servo.Servo(pca.channels[3])

steer = 0
speed = 0
left_limit = -40
right_limit = 40

def turn_left(steer):
    fl_motor.angle = 90 + steer
    fr_motor.angle = 90 - steer
    bl_motor.angle = 90 + steer
    br_motor.angle = 90 - steer

def turn_right(steer):
    fl_motor.angle = 90 + steer
    fr_motor.angle = 90 - steer
    bl_motor.angle = 90 + steer
    br_motor.angle = 90 - steer
    
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

pca.deinit()
