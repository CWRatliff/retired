#robot driver from keyboard lines 181204
import time
import motor_driver
''''
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
br_motor = servo.Servo(pca.channels[0], actuation_range=119, min_pulse=700, max_pulse=2300)
fr_motor = servo.Servo(pca.channels[1])
fl_motor = servo.Servo(pca.channels[2])
bl_motor = servo.Servo(pca.channels[3])

rc = Roboclaw("/dev/ttyS0",115200)
i = rc.Open()
if i != 1:
	print("open status = ["+str(i)+"]")
'''

robot = motor_driver()

steer = 0			# degrees clockwise
speed = 0			# percentage
left_limit = -35
right_limit = 35
#d1 = 7.254
#d2 = 10.5
#d3 = 10.5
#d4 = 10.073
''''
def turn_motor(address, v, av1, av2):
	if v >= 0:
		rc.ForwardM1(address, int(v * av1))
		rc.ForwardM2(address, int(v * av2))
	else:
		rc.BackwardM1(address, int(v * av1))
		rc.BackwardM2(address, int(v * av2))

def stop_all():
	speed = 0
	turn_motor(0X80, 0, 0, 0)
	turn_motor(0X81, 0, 0, 0)
	turn_motor(0X82, 0, 0, 0)

# based on speed & steer, command all motors
def motor():
	vel = speed * 1.27						#roboclaw speed limit -127 to 127
	if steer != 0:							#if steering angle not zero, compute angles, wheel speed
		angle = math.radians(abs(steer))
		ric = d3 / math.sin(angle)			#turn radius - inner corner
		rm = ric * math.cos(angle) + d1		#body central radius
		roc = math.sqrt((rm + d1)**2 + d3**2) #outer corner
		rmo = rm + d4						#middle outer
		rmi = rm - d4						#middle inner
		phi = math.degrees(math.asin(d3 / roc))
		voc = roc / rmo						#velocity corners & middle inner
		vic = ric / rmo
		vim = rmi / rmo
		print("speed, steer:" + str(speed) + " " + str(steer))
		print("ric:"+str(ric)+" rm "+ str(rm)+" roc "+str(roc) + " rmo " + str(rmo) + " rmi " + str(rmi))
		print("phi "+str(phi))

# left turn
	if steer < 0:
		fl_motor.angle = 90 + steer
		fr_motor.angle = 90 + phi
		bl_motor.angle = 90 - steer
		br_motor.angle = 90 - phi
		print(str(vel)+" "+str(vic)+" "+str(speed))
		turn_motor(0x80, vel, vic, vim)
		turn_motor(0x81, vel, vic, voc)
		turn_motor(0x82, vel, 1, voc)

#right turn
	elif steer > 0:
		fl_motor.angle = 90 + phi
		fr_motor.angle = 90 + steer
		bl_motor.angle = 90 - phi
		br_motor.angle = 90 - steer
		turn_motor(0x80, vel, voc, 1)
		turn_motor(0x81, vel, voc, vic)
		turn_motor(0x82, vel, vim, vic)

#straight ahead
	else:
		fl_motor.angle = 90
		fr_motor.angle = 90
		bl_motor.angle = 90
		br_motor.angle = 90
		turn_motor(0x80, vel, 1, 1)
		turn_motor(0x81, vel, 1, 1)
		turn_motor(0x82, vel, 1, 1)
		''''
#main loop
while True:
    key = input("...")
    if key != "":							#repeat?
        cmd = key
    print("cmd="+cmd)
    
    if cmd == 'j':
        print('left')
        if steer > left_limit:
            steer -= 1
            robot.motor(speed, steer)
 ''''
    elif cmd == 'J':
        print('LEFT')
        if steer > left_limit - 3:
            steer -= 3
            motor()

    elif cmd == 'k':
        print('right')
        if steer < right_limit:
            steer += 1
            motor()
    
    elif cmd == 'K':
        print('RIGHT')
        if steer < right_limit + 3:
            steer += 3
            motor()
    
    elif cmd == ' ':
        print('stop')
        steer = 0
        speed = 0
        stop_all()

    elif cmd == 'f':
        if speed < 90:
            speed += 10
            motor()

    elif cmd == 'r':
        if speed > -90:
            speed -= 10
            motor()
	
    elif cmd == '/':
            steer = 0
            motor()

    elif cmd == 'q' or cmd == 'Q':
        break
''''
stop_all()   
pca.deinit()
