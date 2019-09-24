#interface via Xbee radios
# using SPI instead of I2C
#190621 - added compass following
#190715 - strengthened xbee input validation
#190720 - improved compass following
#190802 - U turns
#190816 - GPS, waypoints
#190826 - routes, compass corrections

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
C - GPS corrections
D - one digit commands
E - 'star' commnds
F - 'pound' commands
L - Lat/Lon
O - compass heading
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
azimuth = 0
compass_adjustment = 257
#latsec = 0.0
#lonsec = 0.0
X_pos = 0.0                            # rover location
y_pos = 0.0
x_raw = 0.0
y_raw = 0.0
#startlat = 0.0
#startlon = 0.0
#clatsec = 0.0
#clonsec = 0.0
#oclatsec = 0.0
#oclonsec = 0.0
#latcor = 0.0
#loncor = 0.0
x_start = 0.0
y_start = 0.0
x_delta = 0.0                           # correction to on-board GPS (feet)
y_delta = 0.0
x_corr = 0.0                            # on_board GPS with corrections
y_corr= 0.0
ox_corr = 0.0
oy_corr = 0.0
latitude = math.radians(34.24)          # lat of Camarillo
latfeet = 6076.0/60.0
lonfeet = -latfeet * math.cos(latitude)
lonshort = math.cos(latitude)
ftpersec = 6076.0/60
aftpersec = ftpersec * lonshort

left = False
left_limit = -36
right_limit = 36
epoch = time.time()
wstr = ""
cbuff = ""
flag = False

rteflag = False
rtseg = 0
routes = [[0,0],                    #0
[12, 10, 0],                        #1
[11, 10, 11, 0],                    #2
[14, 15, 16, 17, 0],                #3
[0]]
          
wptdist = 0.0
wptflag = False
waypts=[[0,1],[1,2],[2,3],[3,4],[4,5],[5,6],[6,7],[7,8],[8,9],[9,10],
[22.678, 9.399],                    #10 open area near main gate
[20.808, 7.730, "speed bump"],      #11 mid speed bump
[20.641, 7.396],                    #12 center parking 'T' seam
[20.945, 6.375, "gar door"],        #13
[20.987, 6.066, "driveway center"], #14
[20.830, 5.945, "gravel"],          #15
[20.200, 5.556, "woodpile"],        #16
[19.372, 6.355, "stairs pivot"],    #17
[19.071, 6.840, "shed #4"],         #18
[18.393, 6.283, "longe center"],    #19
[18.181, 7.900, "stall ctr"],       #20
[21.174, 6.110, "E dway start"],    #21
[21.675, 6.576, "tool shed area"],  #22
[22.173, 6.837, "canopy c/l"],      #23
[22.599, 7.159, "EF east entry"],   #24
[11,12]]

robot = motor_driver.motor_driver()

print("Rover 1.0 190904")

#===================================================================
#compute distance from a point to a line
# dist is + if L1P rotates left into L1L2, else negative
def pointline(x1, y1, x2, y2, xp0, yp0, llen):
    dely = y2 - y1
    delx = x2 - x1
    dist = abs(dely*xp0 - delx*yp0 + y2*x1 - x2*y1) / llen
    return (dist)
#===================================================================
#compute distance on flat earth
def distto(x0, y0, x1, y1):
    dely = (la1 - la0)
    delx = (lo0 - lo1)
    dist = math.sqrt(delx**2 + dely**2)
    return(dist)
#===================================================================
#compute angle from point to point on flat earth
def fromto(x0, y0, x1, y1):
    delx = x1 - x0
    dely = y1 - y0
    ang = math.atan2(dely, delx * lonshort)
    return((ang)

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
#=================================================================


cstr = "{aStby}"
spisend(cstr)
cstr = "{d----}"
spisend(cstr)
cstr = "{c----}"
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
                print("msg: " + ts + cbuff)
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
                        if (xchr == 'T'):
                            y_delta = x * latfeet
                        elif xchr == 'N':
                            x_delta = x * lonfeet
                        print ("L/L corr:"+str(x_delta) + "/"+ str(y_delta))

                    except ValueError:
                        print("bad data" + cbuff)

#======================================================================
# single digit keypad commands
                 if (xchr == 'D'):                            
                    xchr = cbuff[2]

                    if xchr == '0':                     # 0 - stop
                        speed = 0
                        robot.motor(speed, steer)

                    elif xchr == '1':                   # 1 - Left
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
                    elif (auto and xchr == '1'):      #left 90 deg
                        azimuth -= 90
                        azimuth %= 360
                    elif (xchr == '2'):               #autopilot on
                        auto = True
                        azimuth = hdg
                        cstr = "{aAuto}"
                        spisend(cstr)
                    elif (auto and xchr == '3'):      #right 90 deg
                        azimuth += 90
                        azimuth %= 360
                    elif (xchr == '4'):               #adj compass
                        compass_adjustment -= 1
                    elif (xchr == '6'):               #adj compass
                        compass_adjustment += 1
                    elif (auto and xchr == '7'):      #left 180 deg
                        left = True
                        azimuth -= 180
                        azimuth %= 360
                    elif (auto and xchr == '9'):      #right 180 deg
                        left = False
                        azimuth += 180
                        azimuth %= 360

#======================================================================
#Keypad commands preceeded by a #
                 elif xchr == 'F':                   #goto waypoint
                    try:
                        wpt = int(cbuff[2:4])
                        if wpt == 0:
                            wptflag = False
                            auto = False
                            cstr = "{aStby}"
                            spisend(cstr)
                            cstr = "{d----}"
                            spisend(cstr)
                            cstr = "{c---}"
                            spisend(cstr)
                            speed = 0
                            steer = 0
                            robot.motor(speed, steer)
                        elif (wpt >0 and wpt < 4):
                            route = wpt
                            rteflag= True
                            rtseg = 0
                            wptflag = True
                            firstwpt = routes[wpt][0]
                            wpt = firstwpt
                        if (wpt >= 10 and wpt <= 24):
                            y_start = y_corr
                            x_start = x_corr
                            y_dest = waypts[wpt][0] * latfeet
                            x_dest = waypts[wpt][1] * lonfeet
                            print ("wpt: "+ str(wpt)+','+str(x_dest)+','+str(y_dest))
                            azimuth = fromto(x_start, y_start, x_dest, y_dest)
                            wptdist = distto(x_start, y_start, x_dest, y_dest)
                            auto = True
                            wptflag = True
                            cstr = "{aWp" + str(wpt) + "}"
                            spisend(cstr)
                    except ValueError:
                        print("bad data" + cbuff)

#======================================================================
                 elif xchr == 'L':                   #lat/long input
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'T'):
                            y_raw = x * latfeet
                            y_corr = y_raw + y_delta
                        elif xchr == 'N':
                            x_raw = x * lonfeet
                            x_corr = x_raw + x_delta
                        wstr = "(x, y):%5.3f/%5.3f" % (x_corr, y_corr)
                        print (wstr)
                        if wptflag:
                            nowhdg = fromto(x_corr, y_corr, x_dest, y_dest)
                            cstr = "{c%3d}" % nowhdg
                            spisend (cstr)
                            print(cstr)
                            azimuth = nowhdg

                    except ValueError:
                        print("bad data" + cbuff)
                    finally:
                        if (y_corr != oy_corr):
                            cstr = "{lt%5.3f}" % y_corr
                            spisend(cstr)
                            oy_corr = y_corr
                        if (x_corr != ox_corr):
                            cstr = "{ln%5.3f}" % x_corr
                            spisend(cstr)
                            ox_corr = x_corr

 #======================================================================
                 if (auto):                          #adjust for > 180 turning
                            
                    if wptflag:
                        dtg = distto(x_corr, y_corr, x_dest, y_dest)
                        cstr = "{d%5.1f}" % dtg
                        spisend(cstr)
                        print(cstr)
                        
                        if (dtg < 3.0):             # a foot from waypoint
                             if rteflag:
                                rtseg += 1
                                wpt = routes[route][rtseg]
                                if (wpt == 0):
                                    cstr = "{Stby}"
                                    spisend(cstr)
                                    wptflg = False
                                    rteflag = False
                                    speed = 0
                                y_start = y_corr         #dup'ed code
                                x_start = x_corr
                                y_dest = waypts[wpt][0] * latfeet
                                x_dest = waypts[wpt][1] * lonfeet
                                print ("wpt: "+ str(wpt) + ','+str(x_dest)+','+str(y_dest))
                                azimuth = fromto(x_start, y_start, x_dest, y_dest)
                                wptdist = distto(x_start, y_start, x_dest, y_dest)
                                cstr = "{aWp" + str(wpt) + "}"
                                spisend(cstr)

                             else:
                                cstr = "{aStby}"
                                spisend(cstr)
                                wptflag =  False

                        if (rteflag or wptflag):
                            nowhdg = fromto(x_corr, y_corr, x_dest, y_dest)
                            angle = nowhdg - azimuth
                            dst = pointline(x_start, y_start, \
                                x_dest, y_dest, x_corr, y_corr, wptdist) 
                            if (dst > 3 or angle > 3):
                                compass_adjustment -= 1
                            if (dst < -3 or angle < -3):
                                compass_adjustment += 1
                    steer = int(azimuth - hdg)
                    if (steer < -180):
                        steer = steer + 360
                    elif (steer > 180):
                        steer = steer - 360
                    if (abs(steer) == 180):
                        if left:
                            steer = -180
                        else:
                            steer = 180
                    robot.motor(speed, steer)
                        
                 if (hdg != oldhdg):
                    cstr = "{h%3d}" % hdg
                    spisend(cstr)
                    oldhdg = hdg
                    print(cstr)
                 if (speed != oldspeed):
                    cstr = "{v%4d}" % speed
                    spisend(cstr)
                    oldspeed = speed
                    print(cstr)
                 if (steer != oldsteer):
                    cstr = "{s%4d}" % steer
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
