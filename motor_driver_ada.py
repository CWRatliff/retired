#rover motor driver class - 4 servo motors for steering, 6 DC motors for locomotion
#190721 steering limits into this module
#200305 changed to new adarfuit servo class
#200404 used 'D' hubs and individual biases
#200405 corrected actual motor to port mapping
#N.B. RC 0x81 & 0x82 may be exchenged for 'Spot 2'

from adafruit_servokit import ServoKit
kit = ServoKit(channels = 16)

import serial
import math
from roboclaw import Roboclaw



class motor_driver_ada:

    def __init__(self, log):
        self.lfbias = 65        # experimentally determined for 'Spot 2'
        self.lrbias = 60
        self.rrbias = 57
        self.rfbias = 61
        self.left_limit = -36
        self.right_limit = 36
        self.d1 = 7.254         #C/L to corner wheels
        self.d2 = 10.5          #mid axle to fwd axle
        self.d3 = 10.5          #mid axle to rear axle
        self.d4 = 10.073        #C/L to mid wheels
        
        self.rr_motor = kit.servo[0]
        self.rf_motor = kit.servo[1]
        self.lf_motor = kit.servo[2]
        self.lr_motor = kit.servo[3]
        self.rr_motor.actuation_range = 120
        self.rf_motor.actuation_range = 120
        self.lf_motor.actuation_range = 120
        self.lr_motor.actuation_range = 120
        self.rr_motor.set_pulse_width_range(700, 2300)
        self.rf_motor.set_pulse_width_range(700, 2300)
        self.lf_motor.set_pulse_width_range(700, 2300)
        self.lr_motor.set_pulse_width_range(700, 2300)
        self.log = log
        
        self.rc = Roboclaw("/dev/ttyS0",115200)
        i = self.rc.Open()

        self.lf_motor.angle = self.rfbias
        self.rf_motor.angle = self.lfbias
        self.lr_motor.angle = self.lrbias
        self.rr_motor.angle = self.rrbias
        self.stop_all()

    def diag(self):
        print("servo rr ="+str(self.rr_motor.angle))
        print("servo rf ="+str(self.rf_motor.angle))
        print("servo lf ="+str(self.lf_motor.angle))
        print("servo lr ="+str(self.lr_motor.angle))
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
        if (steer < self.left_limit):
            steer = self.left_limit
        if (steer > self.right_limit):
            steer = self.right_limit
#        vel = speed * 1.27
        vel = speed * 1.26
        voc = 0
        vic = 0
        #roboclaw speed limit 0 to 127
        # see BOT-2/18 (181201)
        # rechecked 200329
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
            self.lf_motor.angle = self.lfbias - steer
            self.rf_motor.angle = self.rfbias - phi
            self.lr_motor.angle = self.lrbias + steer
            self.rr_motor.angle = self.rrbias + phi
            self.turn_motor(0x80, vel, voc, 1)          #RC 1 - rf, rm
            self.turn_motor(0x81, vel, vim, vic)        #RC 2 - lm, lf
            self.turn_motor(0x82, vel, voc, vic)        #RC 3 - rr, lr
#            cstr = "v, vout, vin %f %f %f\n" % (vel, voc, vic)
#            self.log.write(cstr)

#right turn
        elif steer > 0:
            self.lf_motor.angle = self.lfbias - phi
            self.rf_motor.angle = self.rfbias - steer
            self.lr_motor.angle = self.lrbias + phi
            self.rr_motor.angle = self.rrbias + steer
            self.turn_motor(0x80, vel, vic, vim)
            self.turn_motor(0x81, vel, 1, voc)
            self.turn_motor(0x82, vel, vic, voc)
#            print("80 vic, vim ",vic,vim)
#            print("81 vic, voc ",vic,voc)
#            print("82 vom, voc ", 1, voc)
#            cstr = "v, vout, vin %f %f %f\n" % (vel, voc, vic)
#            self.log.write(cstr)

#straight ahead
        else:
            self.lf_motor.angle = self.lfbias
            self.rf_motor.angle = self.rfbias
            self.lr_motor.angle = self.lrbias
            self.rr_motor.angle = self.rrbias
            self.turn_motor(0x80, vel, 1, 1)
            self.turn_motor(0x81, vel, 1, 1)
            self.turn_motor(0x82, vel, 1, 1)
#       print("v, vout, vin "+str(vel)+", "+str(voc)+", "+str(vic))
#       self.diag()