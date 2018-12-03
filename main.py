#robot driver from keyboard lines 181202
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
br_motor = servo.Servo(pca.channels[0], actuation_range=119, min_pulse=700, max_pulse=2300)
fr_motor = servo.Servo(pca.channels[1])
fl_motor = servo.Servo(pca.channels[2])
bl_motor = servo.Servo(pca.channels[3])

rc = Roboclaw("/dev/ttyS0",115200)
i = rc.Open()
if i != 1
	print("open status = ["+str(i)+"]")
	
steer = 0			# degrees clockwise
speed = 0			# percentage
left_limit = -35
right_limit = 35
d1 = 7.254
d2 = 10.5
d3 = 10.5
d4 = 10.073

def stop_all():
	speed = 0
	rc.ForwardM1(0x80, 0)					#fl
	rc.ForwardM2(0x80, 0)					#ml
	rc.ForwardM1(0x81, 0)					#bl
	rc.ForwardM2(0x81, 0)					#br
	rc.ForwardM1(0x82, 0)					#mr
	rc.ForwardM2(0x82, 0)					#fr

# based on speed & steer, command all motors
def motor():
	vel = speed * 1.27						#roboclaw speed limit -127 to 127
	if steer != 0:							#if steering angle not zero, compute angles, wheel speed
		angle = math.degrees(abs(steer))
		ric = d3 / math.sin(angle)			#turn radius - inner corner
		rm = ric * math.cos(angle) + d1		#body central radius
		roc = math.sqrt(math.pow(rm + d1, 2) + math.pow(d3, 2)) #outer corner
		rmo = rm + d4						#middle outer
		rmi = rm - d4						#middle inner
		outer_angle = math.radians(math.asin(d3 / roc))
		voc = roc / rmo						#velocity corners & middle inner
		vic = ric / rmo
		vim = rmi / rmo

# left turn
	if steer < 0:
		fl_motor.angle = 90 + steer
		fr_motor.angle = 90 + outer_angle
		bl_motor.angle = 90 - steer
		br_motor.angle = 90 - outer_angle
		rc.ForwardM1(0x80, vel * vic)		#fl
		rc.ForwardM2(0x80, vel * vim)		#ml
		rc.ForwardM1(0x81, vel * vic)		#bl
		rc.ForwardM2(0x81, vel * voc)		#br
		rc.ForwardM1(0x82, vel)				#mr
		rc.ForwardM2(0x82, vel * voc)		#fr

#right turn
	elif steer > 0:
		fl_motor.angle = 90 + outer_angle
		fr_motor.angle = 90 + steer
		bl_motor.angle = 90 - outer_angle
		br_motor.angle = 90 - steer
		rc.ForwardM1(0x80, vel * voc)		#fl
		rc.ForwardM2(0x80, vel)				#ml
		rc.ForwardM1(0x81, vel * voc)		#bl
		rc.ForwardM2(0x81, vel * vic)		#br
		rc.ForwardM1(0x82, vel * vim)		#mr
		rc.ForwardM2(0x82, vel * vic)		#fr

#straight ahead
	else:
		fl_motor.angle = 90
		fr_motor.angle = 90
		bl_motor.angle = 90
		br_motor.angle = 90
		rc.ForwardM1(0x80, vel)				#fl
		rc.ForwardM2(0x80, vel)				#ml
		rc.ForwardM1(0x81, vel)				#bl
		rc.ForwardM2(0x81, vel)				#br
		rc.ForwardM1(0x82, vel)				#mr
		rc.ForwardM2(0x82, vel)				#fr
		
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
            motor()
 
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

stop_all()   
pca.deinit()