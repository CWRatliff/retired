#PlayStation interface via Xbee radios
# using SPI instead of I2C
#190515

import sys
import time
#import smbus
import spidev
#import motor_driver

#addr = 0x08
#bus = smbus.SMBus(1)
spi = spidev.SpiDev()
rc = spi.open(0, 0)
spi.max_speed_hz = 125000
time.sleep(0.005)
NUL = [0x00]
spi.xfer(NUL)

steer = 0
speed = 0
accel = 1
left_limit = -35
right_limit = 35
epoch = time.time()
wstr = ""
cbuff = ""

#robot = motor_driver.motor_driver()

#===================================================================
# send chars to slave via SPI
def spisend(cmd):
    i = 0
    while True:
        d = cmd[i]
        c = [ord(d)]
        spi.xfer(c)
        if (d == '}'):                  #end of string
            spi.xfer(NUL)               # flush last char
            break
        i += 1
        #endwhile
    #enddef
        
#while True:                             #purge xbee
#    chr = int(bus.read_byte(addr))
#    if (chr == 0):
#        break
#=================================================================
while True:

    while True:                         # read characters from slave
        c = spi.xfer(NUL)
        if (c[0] == 0 or c[0] == 255):
            break
        d = chr(c[0])
        if (d == '{'):
            cbuff = "{"
            continue
        cbuff += d
        if (d == '}'):
            flag = True
            break
        #endwhile

    if (flag):
        xchr = cbuff[1]    
        if (xchr == 42):
           print("ping")
           epoch = time.time()
           
        if (xchr >= 'A') and (xchr <= 'Z'):

            if xchr == 81:			#Q
                sys.exit()
                
            if xchr == 70:
                 if speed <= 90:
                    speed += (10 * accel)
                    accel = 1
 #                   robot.motor(speed, steer)

            elif xchr == 66:                 #B
                if speed >= -90:
                    speed -= (10 * accel)
                    accel = 1
 #                   robot.motor(speed, steer)

            elif xchr == 76:                 #L
                if steer > left_limit:
                    steer -= (2 * accel)
                    accel = 1
#                    robot.motor(speed, steer)

            elif xchr == 82:                 #R
                if steer < right_limit:
                    steer += (2 * accel)
                    accel = 1
#                    robot.motor(speed, steer)

            elif xchr == 71:                #GEE
                dt = 1
                if steer < (right_limit - 1):
                    while (steer < right_limit):
                        steer += dt
#                        robot.motor(speed, steer)
    #                    time.sleep(0.05)
                steer = right_limit
#                robot.motor(speed, steer)

            elif xchr == 72:                #HAW
                dt = 1
                if steer > (left_limit + 1):
                    while (steer > left_limit):
                        steer -= dt
#                        robot.motor(speed, steer)
    #                    time.sleep(0.05)
                steer = left_limit
#                robot.motor(speed, steer)

            elif xchr == 83:                 #S
                speed = 0
#                robot.motor(speed, steer)

            elif xchr == 90:                 #Z
                dt = 1
                if steer > 0:
                    dt = -1
                    while abs(steer) > 1:
                        steer += dt
#                        robot.motor(speed, steer)
    #                    time.sleep(0.05)
                steer = 0
#               robot.motor(speed, steer)

            elif xchr == 88:                #X exit Select button
#                robot.stop_all()
                speed = 0
                exit()
                
            elif xchr == 'O':					#O - orientation esp hdg from arduino
                eos = cbuff.find('}')
                hdg = int(cbuff[2:eos])
                print("heading = " + str(hdg))

            print("Motor speed, steer "+str(speed)+", "+str(steer))
            cstr = "{v"+str(speed)+','+str(steer)+','+str(hdg)+'}'
            spisend(cstr)

            epoch = time.time()
            #end if =======================

        if (time.time() > (epoch + 1.1)):
    #        print(time.time(),5)
    #        print(epoch,5)
#            robot.stop_all()
            speed = 0;
            
        # end loop ========================
    flag = False
    cbuff = ""
    #end if flag
#    robot.deinit()
