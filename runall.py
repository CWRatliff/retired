import time
import serial
import math
from roboclaw import Roboclaw

rc = Roboclaw("/dev/ttyS0",115200)
i = rc.Open()
print("open status = ["+str(i)+"]")

enc1=rc.ReadEncM1(0x80)
enc2=rc.ReadEncM2(0x80)
enc3=rc.ReadEncM1(0x81)
enc4=rc.ReadEncM2(0x81)
enc5=rc.ReadEncM1(0x82)
enc6=rc.ReadEncM2(0x82)
rc.ForwardM1(0x80, 100);
rc.ForwardM2(0x80, 100);
rc.ForwardM1(0x81, 100);
rc.ForwardM2(0x81, 100);
rc.ForwardM1(0x82, 100);
rc.ForwardM2(0x82, 100);

time.sleep(5)

enc1e=rc.ReadEncM1(0x80)
enc2e=rc.ReadEncM2(0x80)
enc3e=rc.ReadEncM1(0x81)
enc4e=rc.ReadEncM2(0x81)
enc5e=rc.ReadEncM1(0x82)
enc6e=rc.ReadEncM2(0x82)
print("encs:", enc1,enc2,enc3,enc4,enc5, enc6)
print("encs:", enc1e,enc2e,enc3e,enc4e,enc5e, enc6e)
rc.ForwardM1(0x80, 0);
rc.ForwardM2(0x80, 0);
rc.ForwardM1(0x81, 0);
rc.ForwardM2(0x81, 0);
rc.ForwardM1(0x82, 0);
rc.ForwardM2(0x82, 0);
