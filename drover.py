#interface via Xbee radios
# using SPI instead of I2C
#190621 - added compass following
#190715 - strengthened xbee input validation
#190720 - improved compass following
#190802 - U turns
#190816 - GPS, waypoints
#190826 - routes, compass corrections
#190925 - EKF added

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
#C - GPS corrections
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
import cEKF

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
latsec = 0.0
lonsec = 0.0
startlat = 0.0
startlon = 0.0
clatsec = 0.0                           # corrected lat/lon
clonsec = 0.0
oclatsec = 0.0
oclonsec = 0.0
latcor = 0.0                            # corrections
loncor = 0.0
latitude = math.radians(34.24)          # Camarillo
latfeet = 6076.0/60
lonfeet = -latfeet * math.cos(latitude)
spdfactor = .0035
d1 = 7.254
d3 = 10.5

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
[20.641, 7.396, "T seam"],          #12 center parking 'T' seam
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
Kfilter = cEKF.Kalman_filter()
print("Rover 1.0 191015")

#===================================================================
#compute distance from a point to a line
# dist is + if L1P rotates left into L1L2, else negative
def pointline(la1, lo1, la2, lo2, lap0, lop0, llen):
    aa1 = la1 * latfeet 
    ao1 = lo1 * lonfeet
    aa2 = la2 * latfeet
    ao2 = lo2 * lonfeet
    aap0 = lap0 * latfeet
    aop0 = lop0 * lonfeet
    dely = aa2 - aa1
    delx = ao2 - ao1
    dist = abs(dely*aop0 - delx*aap0 + ao2*aa1 - aa2*ao1) / llen
    return (dist)
#===================================================================
#compute distance from lat/lon point to point on flat earth
def distto(la0, lo0, la1, lo1):
    dely = (la1 - la0) * latfeet
    delx = (lo0 - lo1) * lonfeet
    dist = math.sqrt(delx**2 + dely**2)
    return(dist)
#===================================================================
#compute angle from lat/lon point to point on flat earth
def fromto(la0, lo0, la1, lo1):
    delx = lo0 - lo1            #long is -x direction
    dely = la1 - la0            #lat is +y
    ang = math.atan2(dely, delx * math.cos(latitude))
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
'''
                 if (xchr == 'C'):               #lat/long corrections
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'T'):
                            latcor = x
                        elif xchr == 'N':
                            loncor = x
                        print ("L/L corr:"+str(latcor) + "/"+ str(loncor))

                    except ValueError:
                        print("bad data" + cbuff)
'''
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
                            startlat = latsec
                            startlon = lonsec
                            destlat = waypts[wpt][0]
                            destlon = waypts[wpt][1]
                            print ("wpt: "+ str(wpt)+','+str(destlat)+','+str(destlon))
                            azimuth = fromto(startlat, startlon, destlat, destlon)
                            wptdist = distto(startlat, startlon, destlat, destlon)
                            auto = True
                            wptflag = True
                            cstr = "{aWp" + str(wpt) + "}"
                            spisend(cstr)
                            Kfilter.Kalman_start(time.time(), clonsec * lonfeet, \
                                clatsec * latfeet, (math.radians(450-hdg) % 360), \
                                speed * spdfactor)
                    except ValueError:
                        print("bad data" + cbuff)

#======================================================================
                 elif xchr == 'L':                   #lat/long input
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'T'):
                            latsec = x
                            clatsec = latsec + latcor
                        elif xchr == 'N':
                            lonsec = x
                            clonsec = lonsec + loncor
                        wstr = "Lat/Lon:%5.3f/%5.3f" % (clatsec, clonsec)
                        print (wstr)
                        if wptflag:
                            nowhdg = fromto(clatsec, clonsec, destlat, destlon)
                            cstr = "{c%3d}" % nowhdg
                            spisend (cstr)
                            print(cstr)
                            azimuth = nowhdg

                    except ValueError:
                        print("bad data" + cbuff)
                    finally:
                        if (clatsec != oclatsec):
                            cstr = "{lt%5.3f}" % clatsec
                            spisend(cstr)
                            oclatsec = clatsec
                        if (clonsec != oclonsec):
                            cstr = "{ln%5.3f}" % clonsec
                            spisend(cstr)
                            oclonsec = clonsec

 #======================================================================
                 if (auto):                          #adjust for > 180 turning
                            
                    if wptflag:
                        v = speed * spdfactor
                        if (steer = 0):
                            omega = 0
                        else:
                            alpha = math.radians(steer)
                            h = d3/math.sin(alpha)
                            turn = (h * math.cos(alpha) + d1) / 12.0
                            omega = v /turn
                        xEst = Kfilter.Kalman_step(time.time(), clonsec * lonfeet, \
                                clatsec * latfeet, omega, v)
                        clonsec = xEst[0, 0] / lonfeet
                        clatsec = xEst[1, 0] / latfeet
                        dtg = distto(clatsec, clonsec, destlat, destlon)
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
                                startlat = latsec         #dup'ed code
                                startlon = lonsec
                                destlat = waypts[wpt][0]
                                destlon = waypts[wpt][1]
                                print ("wpt: "+ str(wpt) + ','+str(destlat)+','+str(destlon))
                                azimuth = fromto(startlat, startlon,\
                                    destlat, destlon)
                                wptdist = distto(startlat, startlon, \
                                    destlat, destlon)
                                cstr = "{aWp" + str(wpt) + "}"
                                spisend(cstr)

                             else:
                                cstr = "{aStby}"
                                spisend(cstr)
                                wptflag =  False

                        if (rteflag or wptflag):
                            nowhdg = fromto(clatsec, clonsec, destlat, destlon)
                            angle = nowhdg - azimuth
                            dst = pointline(startlat, startlon, \
                                destlat, destlon, clatsec, clonsec, wptdist) 
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
