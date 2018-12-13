#rover motor driver class - 4 servo motors for steering, 6 DC motors for locomotion

from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

#import serial
import math
import time
#from roboclaw import Roboclaw


i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50
br_motor = servo.Servo(pca.channels[0], actuation_range=119, min_pulse=700, max_pulse=2300)
fr_motor = servo.Servo(pca.channels[1], actuation_range=119, min_pulse=700, max_pulse=2300)
fl_motor = servo.Servo(pca.channels[2], actuation_range=119, min_pulse=700, max_pulse=2300)
bl_motor = servo.Servo(pca.channels[3], actuation_range=119, min_pulse=700, max_pulse=2300)
time.sleep(1)
#		rc = Roboclaw("/dev/ttyS0",115200)
#		i = rc.Open()

d1 = 7.254
d2 = 10.5
d3 = 10.5
time.sleep(1)

print("servo br ="+str(br_motor.angle))
print("servo fr ="+str(fr_motor.angle))
print("servo fl ="+str(fl_motor.angle))
print("servo bl ="+str(bl_motor.angle))
#		turn_motor(0x80, vel, voc, 1)
fl_motor.angle = 90
fr_motor.angle = 90
bl_motor.angle = 90
br_motor.angle = 90
	
print("servo br ="+str(br_motor.angle))
print("servo fr ="+str(fr_motor.angle))
print("servo fl ="+str(fl_motor.angle))
print("servo bl ="+str(bl_motor.angle))
'''
	def turn_motor(self, address, v, av1, av2):
		if v >= 0:
			rc.ForwardM1(address, int(v * av1))
			rc.ForwardM2(address, int(v * av2))
		else:
			rc.BackwardM1(address, int(abs(v * av1)))
			rc.BackwardM2(address, int(abs(v * av2)))

	def stop_all(self):
		turn_motor(0X80, 0, 0, 0)
		turn_motor(0X81, 0, 0, 0)
		turn_motor(0X82, 0, 0, 0)

# based on speed & steer, command all motors
	def motor(self, speed, steer):
		print("Motor speed, steer "+str(speed)+", "+str(steer))
		vel = speed * 1.27								#roboclaw speed limit -127 to 127
		if steer != 0:									#if steering angle not zero, compute angles, wheel speed
			angle = math.radians(abs(steer))
			ric = d3 / math.sin(angle)				#turn radius - inner corner
			rm = ric * math.cos(angle) + d1		#body central radius
			roc = math.sqrt((rm+d1)**2 + d3**2) #outer corner
			rmo = rm + d4							#middle outer
			rmi = rm - d4							#middle inner
			phi = math.degrees(math.asin(d3 / roc))
			if steer < 0:
                                phi = -phi
			voc = roc / rmo								#velocity corners & middle inner
			vic = ric / rmo
			vim = rmi / rmo

# left turn
		if steer < 0:
			fl_motor.angle = 90 + steer
			fr_motor.angle = 90 + phi
			bl_motor.angle = 90 - steer
			br_motor.angle = 90 - phi
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
#		diag()
'''
