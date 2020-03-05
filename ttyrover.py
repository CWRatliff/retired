#interface via Xbee radios
# using SPI instead of I2C
#190621 - added compass following
#190715 - strengthened xbee input validation
#190720 - improved compass following
#190802 - U turns
#190816 - GPS, waypoints
#190826 - routes, compass corrections
#190925 - EKF added
#191023 - major code review
#200305 - use serial I/F to Arduino, scratch SPI

'''
+---------+----------+----------+  +---------+----------+----------+
| L 1deg  | Fwd      | R 1deg   |  | L 90deg | Auto     | R 90deg  |
|         |          |          |  |         |          |          |
+---------+----------+----------+  +---------+----------+----------+
| L 5deg  | 0 steer  | R 5deg   |  | Mag     |          | Mag      |
|         |          |          |  | L 1deg  |          | R 1deg   |
+---------+----------+----------+  +---------+----------+----------+
| L 35deg | Rev      | R 35deg  |  | L 180   |          | R 180    |
|         |          |          |  |         |          |          |
+---------+----------+----------+  +---------+----------+----------+
|         | Stop     |          |  |  *      | Stby     |          |
|         |          |          |  |         |          |          |
+---------+----------+----------+  +---------+----------+----------+

Sent codes:
a - status
c - course to wpt
d - distance to wpt
h - heading
lt, ln lat/long
p - pitch
r - roll
s - steering angle
v - speed

Received codes
D - one digit commands
E - 'star' commnds
F - 'pound' commands
O - compass heading
'''
import sys
import serial
import time
import math
import smbus
import motor_driver_ada
import cEKF#rover motor driver class - 4 servo motors for steering, 6 DC motors for locomotion
#190721 steering limits into this module
#200305 changed to new adafruit servo class

from adafruit_servokit import ServoKit
kit = ServoKit(channels = 16)

import serial
import math
from roboclaw import Roboclaw

bias = 60
left_limit = -36
right_limit = 36

class motor_driver:

    def __init__(self):
#        self.i2c = busio.I2C(SCL, SDA)
#        self.pca = PCA9685(self.i2c)
#        self.pca.frequency = 50
#        self.br_motor = servo.Servo(self.pca.channels[0], actuation_range=119, min_pulse=700, max_pulse=2300)
#        self.fr_motor = servo.Servo(self.pca.channels[1], actuation_range=119, min_pulse=700, max_pulse=2300)
#        self.fl_motor = servo.Servo(self.pca.channels[2], actuation_range=119, min_pulse=700, max_pulse=2300)
        self.bl_motor = servo.Servo(self.pca.channels[3], actuation_range=119, min_pulse=700, max_pulse=2300)
        self.br_motor = kit.servo[0]
        self.fr_motor = kit.servo[1]
        self.fl_motor = kit.servo[2]
        self.bl_motor = kit.servo[3]
        self.br_motor.actuation_range = 120
        self.fr_motor.actuation_range = 120
        self.fl_motor.actuation_range = 120
        self.bl_motor.actuation_range = 120
        self.br_motor.set_pulse_width_range(700, 2300)
        self.fr_motor.set_pulse_width_range(700, 2300)
        self.fl_motor.set_pulse_width_range(700, 2300)
        self.bl_motor.set_pulse_width_range(700, 2300)
        
        self.rc = Roboclaw("/dev/ttyS0",115200)
        i = self.rc.Open()

        self.d1 = 7.254         #C/L to corner wheels
        self.d2 = 10.5          #mid axle to fwd axle
        self.d3 = 10.5          #mid axle to rear axle
        self.d4 = 10.073        #C/L to mid wheels

        self.fl_motor.angle = bias
        self.fr_motor.angle = bias
        self.bl_motor.angle = bias
        self.br_motor.angle = bias

    def diag(self):
        print("servo br ="+str(self.br_motor.angle))
        print("servo fr ="+str(self.fr_motor.angle))
        print("servo fl ="+str(self.fl_motor.angle))
        print("servo bl ="+str(self.bl_motor.angle))
#       self.turn_motor(0x80, vel, voc, 1)

    def turn_motor(self, address, v, av1, av2):
        v1 = int(v * av1)
        v2 = int(v * av2)
        if v >= 0:
            self.rc.ForwardM1(address, v1)
            self.rc.ForwardM2(address, v2)
        else:
            self.rc.BackwardM1(address, abs(v1))
            self.rc.BackwardM2(address, abs(v2))
#       print("m1, m2 = "+str(v1)+", "+str(v2))

    def stop_all(self):
        self.turn_motor(0X80, 0, 0, 0)
        self.turn_motor(0X81, 0, 0, 0)
        self.turn_motor(0X82, 0, 0, 0)

    def motor_speed(self):
                speed = self.rc.ReadSpeedM1(0x82)
                print ("motor speed =" + str(speed))
                speed = self.rc.ReadSpeedM2(0x81)
                print ("motor speed =" + str(speed))

# based on speed & steer, command all motors
    def motor(self, speed, steer):
#        print("Motor speed, steer "+str(speed)+", "+str(steer))
        if (steer < left_limit):
            steer = left_limit
        if (steer > right_limit):
            steer = right_limit
#        vel = speed * 1.27
        vel = speed * 1.26
        voc = 0
        vic = 0
        #roboclaw speed limit -127 to 127
        if steer != 0:                                  #if steering angle not zero, compute angles, wheel speed
            angle = math.radians(abs(steer))
            ric = self.d3 / math.sin(angle)             #turn radius - inner corner
            rm = ric * math.cos(angle) + self.d1        #body central radius
            roc = math.sqrt((rm+self.d1)**2 + self.d3**2) #outer corner
            rmo = rm + self.d4                          #middle outer
            rmi = rm - self.d4                          #middle inner
            phi = math.degrees(math.asin(self.d3 / roc))
            if steer < 0:
                                phi = -phi
            voc = roc / rmo                             #velocity corners & middle inner
            vic = ric / rmo
            vim = rmi / rmo

# SERVO MOTORS ARE COUNTER CLOCKWISE
# left turn
        if steer < 0:
            self.fl_motor.angle = bias - steer
            self.fr_motor.angle = bias - phi
            self.bl_motor.angle = bias + steer
            self.br_motor.angle = bias + phi
            self.turn_motor(0x80, vel, vic, vim)
            self.turn_motor(0x81, vel, vic, voc)
            self.turn_motor(0x82, vel, 1, voc)

#right turn
        elif steer > 0:
            self.fl_motor.angle = bias - phi
            self.fr_motor.angle = bias - steer
            self.bl_motor.angle = bias + phi
            self.br_motor.angle = bias + steer
            self.turn_motor(0x80, vel, voc, 1)
            self.turn_motor(0x81, vel, voc, vic)
            self.turn_motor(0x82, vel, vim, vic)

#straight ahead
        else:
            self.fl_motor.angle = bias
            self.fr_motor.angle = bias
            self.bl_motor.angle = bias
            self.br_motor.angle = bias
            self.turn_motor(0x80, vel, 1, 1)
            self.turn_motor(0x81, vel, 1, 1)
            self.turn_motor(0x82, vel, 1, 1)
#       print("v, vout, vin "+str(vel)+", "+str(voc)+", "+str(vic))
#       self.diag()



steer = 0
speed = 0
azimuth = 0
hdg = 0
oldsteer = 500
oldspeed = 500
oldhdg = 500
compass_adjustment = 0
ilatsec = 0.0                           # input from GPS hardware
ilonsec = 0.0
startlat = 0.0
startlon = 0.0
flatsec = 0.0                           # Kalman filtered lat/lon
flonsec = 0.0

latitude = math.radians(34.24)          # Camarillo
latfeet = 6076.0/60
lonfeet = -latfeet * math.cos(latitude)
spdfactor = .0035
d1 = 7.254
d3 = 10.5

left = False
left_limit = -36
right_limit = 36

class motor_driver:

    def __init__(self):
#        self.i2c = busio.I2C(SCL, SDA)
#        self.pca = PCA9685(self.i2c)
#        self.pca.frequency = 50
#        self.br_motor = servo.Servo(self.pca.channels[0], actuation_range=119, min_pulse=700, max_pulse=2300)
#        self.fr_motor = servo.Servo(self.pca.channels[1], actuation_range=119, min_pulse=700, max_pulse=2300)
#        self.fl_motor = servo.Servo(self.pca.channels[2], actuation_range=119, min_pulse=700, max_pulse=2300)
        self.bl_motor = servo.Servo(self.pca.channels[3], actuation_range=119, min_pulse=700, max_pulse=2300)
        self.br_motor = kit.servo[0]
        self.fr_motor = kit.servo[1]
        self.fl_motor = kit.servo[2]
        self.bl_motor = kit.servo[3]
        self.br_motor.actuation_range = 120
        self.fr_motor.actuation_range = 120
        self.fl_motor.actuation_range = 120
        self.bl_motor.actuation_range = 120
        self.br_motor.set_pulse_width_range(700, 2300)
        self.fr_motor.set_pulse_width_range(700, 2300)
        self.fl_motor.set_pulse_width_range(700, 2300)
        self.bl_motor.set_pulse_width_range(700, 2300)
        
        self.rc = Roboclaw("/dev/ttyS0",115200)
        i = self.rc.Open()

        self.d1 = 7.254         #C/L to corner wheels
        self.d2 = 10.5          #mid axle to fwd axle
        self.d3 = 10.5          #mid axle to rear axle
        self.d4 = 10.073        #C/L to mid wheels

        self.fl_motor.angle = bias
        self.fr_motor.angle = bias
        self.bl_motor.angle = bias
        self.br_motor.angle = bias

    def diag(self):
        print("servo br ="+str(self.br_motor.angle))
        print("servo fr ="+str(self.fr_motor.angle))
        print("servo fl ="+str(self.fl_motor.angle))
        print("servo bl ="+str(self.bl_motor.angle))
#       self.turn_motor(0x80, vel, voc, 1)

    def turn_motor(self, address, v, av1, av2):
        v1 = int(v * av1)
        v2 = int(v * av2)
        if v >= 0:
            self.rc.ForwardM1(address, v1)
            self.rc.ForwardM2(address, v2)
        else:
            self.rc.BackwardM1(address, abs(v1))
            self.rc.BackwardM2(address, abs(v2))
#       print("m1, m2 = "+str(v1)+", "+str(v2))

    def stop_all(self):
        self.turn_motor(0X80, 0, 0, 0)
        self.turn_motor(0X81, 0, 0, 0)
        self.turn_motor(0X82, 0, 0, 0)

    def motor_speed(self):
                speed = self.rc.ReadSpeedM1(0x82)
                print ("motor speed =" + str(speed))
                speed = self.rc.ReadSpeedM2(0x81)
                print ("motor speed =" + str(speed))

# based on speed & steer, command all motors
    def motor(self, speed, steer):
#        print("Motor speed, steer "+str(speed)+", "+str(steer))
        if (steer < left_limit):
            steer = left_limit
        if (steer > right_limit):
            steer = right_limit
#        vel = speed * 1.27
        vel = speed * 1.26
        voc = 0
        vic = 0
        #roboclaw speed limit -127 to 127
        if steer != 0:                                  #if steering angle not zero, compute angles, wheel speed
            angle = math.radians(abs(steer))
            ric = self.d3 / math.sin(angle)             #turn radius - inner corner
            rm = ric * math.cos(angle) + self.d1        #body central radius
            roc = math.sqrt((rm+self.d1)**2 + self.d3**2) #outer corner
            rmo = rm + self.d4                          #middle outer
            rmi = rm - self.d4                          #middle inner
            phi = math.degrees(math.asin(self.d3 / roc))
            if steer < 0:
                                phi = -phi
            voc = roc / rmo                             #velocity corners & middle inner
            vic = ric / rmo
            vim = rmi / rmo

# SERVO MOTORS ARE COUNTER CLOCKWISE
# left turn
        if steer < 0:
            self.fl_motor.angle = bias - steer
            self.fr_motor.angle = bias - phi
            self.bl_motor.angle = bias + steer
            self.br_motor.angle = bias + phi
            self.turn_motor(0x80, vel, vic, vim)
            self.turn_motor(0x81, vel, vic, voc)
            self.turn_motor(0x82, vel, 1, voc)

#right turn
        elif steer > 0:
            self.fl_motor.angle = bias - phi
            self.fr_motor.angle = bias - steer
            self.bl_motor.angle = bias + phi
            self.br_motor.angle = bias + steer
            self.turn_motor(0x80, vel, voc, 1)
            self.turn_motor(0x81, vel, voc, vic)
            self.turn_motor(0x82, vel, vim, vic)

#straight ahead
        else:
            self.fl_motor.angle = bias
            self.fr_motor.angle = bias
            self.bl_motor.angle = bias
            self.br_motor.angle = bias
            self.turn_motor(0x80, vel, 1, 1)
            self.turn_motor(0x81, vel, 1, 1)
            self.turn_motor(0x82, vel, 1, 1)
#       print("v, vout, vin "+str(vel)+", "+str(voc)+", "+str(vic))
#       self.diag()
