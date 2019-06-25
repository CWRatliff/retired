#PlayStation interface via Xbee radios
# using SPI instead of I2C
#190621 - added compass following
import sys
import time
import smbus
import spidev
import motor_driver

#addr = 0x08
bus = smbus.SMBus(1)
spi = spidev.SpiDev()
rc = spi.open(0, 0)
spi.max_speed_hz = 125000
time.sleep(0.005)
NUL = [0x00]
spi.xfer(NUL)

steer = 0
speed = 0
hdg = 0
oldsteer = 500
oldspeed = 500
oldhdg = 500
compass = False
comhdg = 0

left_limit = -36
right_limit = 36
epoch = time.time()
wstr = ""
cbuff = ""
flag = False

robot = motor_driver.motor_driver()
#===================================================================
# progressive steering
def turn():
    strang = abs(steer)
    if (strang < 3):
        turnang = 1
    elif (strang < 10):
        turnang = 2
    elif (strang < 20):
        turnang = 3
    else:
        turnang = 4
    return turnang
#===================================================================
# send chars to slave via SPI
def spisend(cmd):
    i = 0
    while True:
        d = cmd[i]                      # d - char
        c = [ord(d)]                    # c - list containing an int
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
try:
    while True:

        while True:                         # read characters from slave
            c = spi.xfer(NUL)
            if (c[0] == 0 or c[0] == 255):
                break
            print(c)
            d = chr(c[0])
            if (d == '{'):
                cbuff = "{"
                continue
            cbuff += d
            if (d == '}'):
                flag = True
                break
            #endwhile

        if (flag):                          # flag means we got a command
            if (len(cbuff) < 3):
                continue
            xchr = cbuff[1]    
            if (xchr == '*'):                   #ping
    #           print("ping")
               epoch = time.time()
    #        print(xchr)
               
            if (xchr >= 'A') and (xchr <= 'Z'):

                if xchr == 'Q':			#Quit (unassigned)
                    sys.exit()
                    
                if xchr == 'F':                     #Forward 10-20% D pad up
                     if speed <= 90:
                        speed += 10
                        robot.motor(speed, steer)

                elif xchr == 'B':                   #B Reverse 10-20% D pad down
                    if speed >= -90:
                        speed -= 10
                        robot.motor(speed, steer)

                elif xchr == 'C':                   #C steer to compass heading
                    compass = True
                    comhdg = hdg

                elif xchr == 'L':                   #Left 2 deg, D pad left
                    if steer > left_limit:
                        steer -= turn()
                        robot.motor(speed, steer)

                elif xchr == 'R':                   #Right 2 deg, D pad right
                    if steer < right_limit:
                        steer += turn()
                        robot.motor(speed, steer)

                elif xchr == 'G':                   #GEE steer right limit, D pad right + L2
                    dt = 1
                    if steer < (right_limit - 1):
                        while (steer < right_limit):
                            steer += dt
                            robot.motor(speed, steer)
        #                    time.sleep(0.05)
                    steer = right_limit
                    robot.motor(speed, steer)

                elif xchr == 'H':                   #HAW steer left limit, D pad right + L2
                    dt = 1
                    if steer > (left_limit + 1):
                        while (steer > left_limit):
                            steer -= dt
                            robot.motor(speed, steer)
        #                    time.sleep(0.05)
                    steer = left_limit
                    robot.motor(speed, steer)

                elif xchr == 'S':                   #Stop drive motors L3 left joystick button
                    speed = 0
                    robot.motor(speed, steer)

                elif xchr == 'Z':                   #Zero steering wheels left upper trigger
                    dt = 1
                    if steer > 0:
                        dt = -1
                        while abs(steer) > 1:
                            steer += dt
                            robot.motor(speed, steer)
        #                    time.sleep(0.05)
                    steer = 0
                    robot.motor(speed, steer)
                    compass = False
                    comhdg = 0

                elif xchr == 'X':                   #X exit Select button
                    robot.stop_all()
                    speed = 0
                    exit()
                    
                elif xchr == 'O':			#O - orientation esp hdg from arduino
                    eos = cbuff.find('}')
                    hdg = int(cbuff[2:eos])
                    print("heading = " + str(hdg))

                    print("Motor speed, steer "+str(speed)+", "+str(steer))

                if (compass):
                    if (comhdg < 180 and hdg > 180):    #seems crude but cheap vs vectors
                        steer = 2
                    elif (comhdg > 180 and hdg < 180):
                        steer = -2
                    elif (comhdg > hdg):
                        steer = 2
                    elif (comhdg < hdg):
                        steer = -2
                        
                    if (steer != 0):
                        robot.motor(speed, steer)
                        
                if (hdg != oldhdg):
                    cstr = "{h"+str(hdg)+"}"
                    spisend(cstr)
                    oldhdg = hdg
                    print(cstr)
                if (speed != oldspeed):
                    cstr = "{v"+str(speed)+"}"
                    spisend(cstr)
                    oldspeed = speed
                    print(cstr)
                if (steer != oldsteer):
                    cstr = "{s"+str(steer)+"}'"
                    spisend(cstr)
                    oldsteer = steer
                    print(cstr)

                epoch = time.time()
                #end if =======================

            if (time.time() > (epoch + 1.1)):
        #        print(time.time(),5)
        #        print(epoch,5)
        #        robot.stop_all()
                speed = 0;
                
            # end loop ========================
        flag = False
        cbuff = ""
        #end if flag

finally:
    robot.motor(0,0)
    print("Halted")
#    robot.deinit()
