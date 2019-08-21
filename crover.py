#interface via Xbee radios
# using SPI instead of I2C
#190621 - added compass following
#190715 - strengthened xbee input validation
#190720 - improved compass following
#190802 - U turns
#190816 - GPS, waypoints

'''
+---------+----------+----------+  +---------+----------+----------+
| L 1deg  | Fwd      | R 1deg   |  | L 90deg | Auto     | R 90deg  |
|         |          |          |  |         |          |          |
+---------+----------+----------+  +---------+----------+----------+
| L 5deg  | 0 steer  | R 5deg   |  |         |          |          |
|         |          |          |  |         |          |          |
+---------+----------+----------+  +---------+----------+----------+
| L 35deg | Rev      | R 35deg  |  | L 180   |          | R 180    |
|         |          |          |  |         |          |          |
+---------+----------+----------+  +---------+----------+----------+
|         | Stop     |          |  |  *      | Stby     |          |
|         |          |          |  |         |          |          |
+---------+----------+----------+  +---------+----------+----------+
'''
import sys
import time
import math
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
auto = False
waypoint = False
azimuth = 0
compass_adjustment = 230
latsec = 0.0
lonsec = 0.0
startlat = 0.0
startlon = 0.0
clatsec = 0.0
clonsec = 0.0
latcor = 0.0
loncor = 0.0
latitude = math.radians(34.24)
ftpersec = 6076.0/60
aftpersec = ftpersec * math.cos(latitude)

left = False
left_limit = -36
right_limit = 36
epoch = time.time()
wstr = ""
cbuff = ""
flag = False

waypts=[[0,1],[1,2],[2,3],[3,4],[4,5],[5,6],[6,7],[7,8],[8,9],[9,10],
[22.678, 9.399],            #10 open area near main gate
[20.808, 7.73],             #11 mid speed bump
[20.641, 7.396],            #12 center parking 'T' seam
[11,12]]

robot = motor_driver.motor_driver()

print("Rover 1.0 190816")

#===================================================================
#compute distance from a point to a line
def pointline(la1, lo1, la2, lo2, lap0, lop0):
    aa1 = la1 * ftpersec 
    ao1 = lo1 * aftpersec
    aa2 = la2 * ftpersec
    ao2 = lo2 * aftpersec
    aap0 = lap0 * ftpersec
    aop0 = lop0 * aftpersec
    dely = aa2 - aa1
    delx = ao2 - ao1
    linedist = math.sqrt(delx**2 + dely**2)
    dist = abs(dely*aop0 - delx*aap0 + ao2*aa1 - aa2*ao1) / linedist
    return (dist)
#===================================================================
#compute distance from lat/lon point to point on flat earth
def distto(la0, lo0, la1, lo1):
    dely = (la1 - la0) * ftpersec
    delx = (lo0 - lo1) * math.cos(latitude) * ftpersec
    dist = math.sqrt(delx**2 + dely**2)
    return(dist)
#===================================================================
#compute angle from lat/lon point to point on flat earth
def fromto(la0, lo0, la1, lo1):
    delx = la1 - la0            #lat is +y
    dely = lo0 - lo1            #long is -x direction
    ang = math.atan2(delx, dely * math.cos(latitude))
    return((450 - math.degrees(ang))%360)

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


cstr = "{aStby}"
spisend(cstr)

try:
    while True:

        while True:                         # read characters from slave
            c = spi.xfer(NUL)
            if (c[0] == 0 or c[0] == 255):
                break
 #           print(c)
            d = chr(c[0])
            if (d == '{'):
                cbuff = "{"
                continue
            cbuff += d
            if (d == '}'):
                flag = True
                tt=time.localtime()
                ts=time.strftime("%H:%M:%S ", tt)
                print("msg; " + ts + cbuff)
                break
            #endwhile

#========================================================================
        if (flag):                          # flag means we got a command
            msglen = len(cbuff)
            if (msglen < 3 or cbuff[0] != '{'):
                cbuff = ""
                continue
            xchr = cbuff[1]    
            if (xchr == '*'):                   #ping
    #           print("ping")
                epoch = time.time()
    #        print(xchr)
               
            if (xchr >= 'A') and (xchr <= 'Z'):

                 if (xchr == 'C'):               #lat/long corrections
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'A'):
                            latcor = x
                        elif xchr == 'O':
                            loncor = x
                        print ("L/L corr:"+str(latcor) + "/"+ str(loncor))

                    except ValueError:
                        print("bad data" + cbuff)

#======================================================================
# single digit keypad commands
                 if (xchr == 'D'):                            
                    xchr = cbuff[2]

                    if xchr == '0':                     # 0 - stop
                        speed = 0
                        robot.motor(speed, steer)

                    if xchr == '1':                     # 1 - Left
                        if (auto):
                            azimuth -= 1
                        else:
                            steer -= turn()
                            robot.motor(speed, steer)
                            
                    elif xchr == '2':                   # 2 - Forward
                        if speed <= 90:
                            speed += 10
                            robot.motor(speed, steer)

                    elif xchr == '3':                   # 3 - Right
                        if (auto):
                            azimuth += 1
                        else:
                            steer += turn()
                            robot.motor(speed, steer)

                    elif xchr == '4':                   # 4 - Left 5 deg
                        if (auto):
                            azimuth -= 5
                        else:
                            steer -= 5
                            robot.motor(speed, steer)
                            
                    elif xchr == '5':                   # 5 - Steer zero
                        steer = 0
                        robot.motor(speed, steer)
                        auto = False
                        azimuth = 0

                    elif xchr == '6':                   # 6 - Left 5 deg
                        if (auto):
                            azimuth += 5
                        else:
                            steer += 5
                            robot.motor(speed, steer)
                            
                    elif xchr == '7':                   # 7 - HAW steer left limi
                        dt = 1
                        if steer > (left_limit + 1):
                            while (steer > left_limit):
                                steer -= dt
                                robot.motor(speed, steer)
        #                    time.sleep(0.05)
                        steer = left_limit
                        robot.motor(speed, steer)
                            
                    elif xchr == '8':                   # 8 -  Reverse
                        if speed >= -90:
                            speed -= 10
                            robot.motor(speed, steer)

                    elif xchr == '9':                   # 9 - GEE steer right limit
                        dt = 1
                        if steer < (right_limit - 1):
                            while (steer < right_limit):
                                steer += dt
                                robot.motor(speed, steer)
        #                    time.sleep(0.05)
                        steer = right_limit
                        robot.motor(speed, steer)
#===================end of D commands

 
                 elif xchr == 'X':                   #X exit Select button
                    robot.stop_all()
                    speed = 0
                    exit()
                    
                 elif xchr == 'O':                   #O - orientation esp hdg from arduino
                    if (msglen < 4 or msglen > 6):
                        cbuff = ""
                        continue
                    hdg = int(cbuff[2:msglen-1])
                    hdg = (hdg + compass_adjustment)%360
                    print("heading = " + str(hdg))

                    print("Motor speed, steer "+str(speed)+", "+str(steer))

#======================================================================
# Keypad commands preceded by a star
                 elif xchr == 'E':
                    xchr = cbuff[2]
                    if (xchr == '0'):               #standby
                        auto = False
                        azimuth = hdg
                        cstr = "{aStby}"
                        spisend(cstr)
                    if (auto and xchr == '1'):      #left 90 deg
                        azimuth -= 90
                        azimuth %= 360
                    if (xchr == '2'):               #autopilot on
                        auto = True
                        azimuth = hdg
                        cstr = "{aAuto}"
                        spisend(cstr)
                    if (auto and xchr == '3'):      #right 90 deg
                        azimuth += 90
                        azimuth %= 360
                    if (auto and xchr == '7'):      #left 180 deg
                        left = True
                        azimuth -= 180
                        azimuth %= 360
                    if (auto and xchr == '9'):      #right 180 deg
                        left = False
                        azimuth += 180
                        azimuth %= 360

#======================================================================
#Keypad commands preceeded by a #
                 elif xchr == 'F':                   #goto waypoint
                    try:
                        wpt = int(cbuff[2:4])
                        if wpt == 0:
                            waypoint = False
                            cstr = "{aAuto}"
                            spisend(cstr)
                        elif (wpt == 10):
                            startlat = latsec
                            startlon = lonsec
                            destlat = waypts[10][0]
                            destlon = waypts[10][1]
                            comhdg = fromto(destlon, destlat, startlon, startlat)
                            auto = True
                            waypoint = True
                            cstr = "{aWp10}"
                            spisend(cstr)
                    except ValueError:
                        print("bad data" + cbuff)

#======================================================================
                 elif xchr == 'L':                   #lat/long input
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'A'):
                            clatsec = latsec + latcor
                            latsec = x
                        elif xchr == 'O':
                            lonsec = x
                            clonsec = lonsec + loncor
                        wstr = "Lat/long:%5.3f/%5.3f" % (clatsec, clonsec)
                        print (wstr)
                        if waypoint:
                            nowhdg = fromto(clatsec, clonsec, destlon, destlat)
                            cstr = "{c%3d}" % nowhdg
                            spisend (cstr)
                            comhdg = nowhdg

                    except ValueError:
                        print("bad data" + cbuff)
                    finally:
                        cstr = "{la%5.3f}" % clatsec
                        spisend(cstr)
                        cstr = "{lo%5.3f}" % clonsec
                        spisend(cstr)

 #======================================================================
                 if (auto):                          #adjust for > 180 turning
                    steer = azimuth - hdg
                    if (steer < -180):
                        steer = steer + 360
                    elif (steer > 180):
                        steer = steer - 360
                    if (abs(steer) == 180):
                        if left:
                            steer = -180
                        else:
                            steer = 180
                          
                    if waypoint:                    # a foot from waypoint
                        dtg = distto(clatsec, clonsec, destlat, destlon)
                        cstr = "{d%5.1f}" % dtg
                        spisend(cstr)
                        if (dst < 1.0):
                            cstr = "{aAuto}"
                            spisend(cstr)
                            waypoint =  False
                            speed = 0
                            
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
                    cstr = "{s"+str(steer)+"}"
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
