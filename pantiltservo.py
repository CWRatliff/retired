#rover motor driver class - 4 servo motors for steering, 6 DC motors for locomotion
#190721 steering limits into this module
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels = 16)

print ("to 90")
kit.servo[0].angle = 90
time.sleep(5)

print ("to 50")
kit.servo[0].angle = 50
time.sleep(5)

print ("to 20")
kit.servo[0].angle = 20

print ("done")
