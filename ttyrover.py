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
import spidev
import motor_driver_ada
import cEKF

steer = 0
speed = 0
azimuth = 0
hdg = 0
oldsteer = 500
oldspeed = 500
oldhdg = 500
compass_adjustment = 257
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
epoch = time.time()

auto = False
flag = False
rteflag = False
wptflag = False

rtseg = 0
routes = [[0,0],                    #0
[12, 10, 0],                        #1
[11, 10, 11, 0],                    #2
[14, 15, 16, 17, 0],                #3
[0]]
          
wptdist = 0.0
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

robot = motor_driver_ada.motor_driver_ada()
Kfilter = cEKF.Kalman_filter()
print("Rover 1.0 191023")
port = "/dev/ttyUSB0"
tty = serial.Serial(port, 9600)
tty.flushInput()

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
#compute compass angle from lat/lon point to point on flat earth
def fromto(la0, lo0, la1, lo1):
    delx = lo0 - lo1            #long is -x direction
    dely = la1 - la0            #lat is +y
    ang = math.atan2(dely, delx * math.cos(latitude))
    return((450 - math.degrees(ang))%360)

#===================================================================
def readusb():
    try:
        d = tty.read(1).decode("utf-8")
        return(d)
    except UnicodeDecodeError:
        print("Woops")
        return (0)

        
#while True:                             #purge xbee
#    chr = int(bus.read_byte(addr))
#    if (chr == 0):
#        break
#=================================================================
#=================================================================

cstr = "{aStby}"
tty.write(cstr.encode("utf-8"))
cstr = "{d----}"
tty.write(cstr.encode("utf-8"))
cstr = "{c----}"
tty.write(cstr.encode("utf-8"))
cbuff = ""

try:
    while True:             #######  main loop    #######
        
        while (tty.inWaiting() > 0):                         # read characters from slave
            d = readusb()
            if (d == 0):
                continue
            if (d == '{'):
                cbuff = ""
                flag = False
            cbuff += d
            if (d == '}'):
                flag = True
                break
            #endwhile read

#========================================================================
        if (flag):                          # flag means we got a command
            msglen = len(cbuff)
            if (msglen < 3 or cbuff[0] != '{'):
                print("bad msg: ", cbuff)
                cbuff = ""
                flag= False
                continue
            tt=time.localtime()
            ts=time.strftime("%H:%M:%S ", tt)
            print("msg: " + ts + cbuff)
            xchr = cbuff[1]    
               
            if (xchr >= 'A') and (xchr <= 'Z'):

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
                            robot.motor(speed, steer)
                            
                    elif xchr == '2':                   # 2 - Forward
                        if speed <= 90:
                            speed += 10
                            robot.motor(speed, steer)

                    elif xchr == '3':                   # 3 - Right
                        if (auto):
                            azimuth += 1
                        else:
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

                    elif xchr == '6':                   # 6 - Left 5 deg
                        if (auto):
                            azimuth += 5
                        else:
                            steer += 5
                            robot.motor(speed, steer)
                            
                    elif xchr == '7':                   # 7 - HAW steer left limit
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
                    if (xchr == '0'):                   #standby
                        auto = False
                        azimuth = hdg
                        cstr = "{aStby}"
                        tty.write(cstr.encode("utf-8"))
                    elif (auto and xchr == '1'):      #left 90 deg
                        azimuth -= 90
                        azimuth %= 360
                    elif (xchr == '2'):               #autopilot on
                        auto = True
                        azimuth = hdg
                        cstr = "{aAuto}"
                        tty.write(cstr.encode("utf-8"))
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
                            rteflag = False
                            auto = False
                            cstr = "{aStby}"
                            tty.write(cstr.encode("utf-8"))
                            cstr = "{d----}"
                            tty.write(cstr.encode("utf-8"))
                            cstr = "{c---}"
                            tty.write(cstr.encode("utf-8"))
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
                            startlat = ilatsec
                            startlon = ilonsec
                            destlat = waypts[wpt][0]
                            destlon = waypts[wpt][1]
                            print ("wpt: "+ str(wpt)+','+str(destlat)+','+str(destlon))
                            azimuth = fromto(startlat, startlon, destlat, destlon)
                            wptdist = distto(startlat, startlon, destlat, destlon)
                            auto = True
                            wptflag = True
                            cstr = "{aWp" + str(wpt) + "}"
                            tty.write(cstr.encode("utf-8"))
                            Kfilter.Kalman_start(time.time(), ilonsec * lonfeet, \
                                ilatsec * latfeet, (math.radians(450-hdg) % 360), \
                                speed * spdfactor)
                    except ValueError:
                        print("bad data" + cbuff)

#======================================================================
                 elif xchr == 'L':                   #lat/long input from GPS h/w
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'T'):
                            ilatsec = x
                        elif xchr == 'N':
                            ilonsec = x

                    except ValueError:
                        print("bad data" + cbuff)
                    finally:
                        pass
 #======================================================================
                 if (auto):
                     
                    if (time.time < (epoch + 1)):
                        epoch = time.time()
                        continue
                    if wptflag:
                        v = speed * spdfactor
                        if (steer == 0):
                            omega = 0
                        else:
                            alpha = math.radians(steer)
                            h = d3/math.sin(alpha)
                            turn = (h * math.cos(alpha) + d1) / 12.0
                            omega = v /turn
                        xEst = Kfilter.Kalman_step(time.time(), ilonsec * lonfeet, \
                                ilatsec * latfeet, omega, v)
                        flonsec = xEst[0, 0] / lonfeet
                        flatsec = xEst[1, 0] / latfeet
                        fhdg= (450 - math.degrees(xEst[2,0]))%360
                        print("filtered L/L:",flatsec, "/",flonsec)
                        print("Filtered hdg: ", fhdg)
                        print("Filtered speed: ",xEst[3,0] / spdfactor)
                        dtg = distto(flatsec, flonsec, destlat, destlon)
                        cstr = "{d%5.1f}" % dtg
                        tty.write(cstr.encode("utf-8"))
                        print(cstr)

                        cstr = "{lt%5.3f}" % flatsec    #send to controller
                        tty.write(cstr.encode("utf-8"))
                        cstr = "{ln%5.3f}" % flonsec
                        tty.write(cstr.encode("utf-8"))
                    
                        if (dtg < 3.0):             # a foot from waypoint
                             if rteflag:
                                rtseg += 1
                                wpt = routes[route][rtseg]
                                if (wpt == 0):
                                    cstr = "{Stby}"
                                    tty.write(cstr.encode("utf-8"))
                                    wptflg = False
                                    rteflag = False
                                    speed = 0
                                startlat = ilatsec         #dup'ed code
                                startlon = ilonsec
                                destlat = waypts[wpt][0]
                                destlon = waypts[wpt][1]
                                print ("wpt: "+ str(wpt) + ','+str(destlat)+','+str(destlon))
                                azimuth = fromto(startlat, startlon,\
                                    destlat, destlon)
                                wptdist = distto(startlat, startlon, \
                                    destlat, destlon)
                                cstr = "{aWp" + str(wpt) + "}"
                                tty.write(cstr.encode("utf-8"))

                             else:
                                cstr = "{aStby}"
                                tty.write(cstr.encode("utf-8"))
                                wptflag =  False

                        if (rteflag or wptflag):
                            nowhdg = fromto(flatsec, flonsec, destlat, destlon)
                            angle = nowhdg - azimuth
                            dst = pointline(startlat, startlon, \
                                destlat, destlon, flatsec, flonsec, wptdist) 
                            if (dst > 3 or angle > 3):
                                compass_adjustment -= 1
                            if (dst < -3 or angle < -3):
                                compass_adjustment += 1
                        #end if auto
                                
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
                    tty.write(cstr.encode("utf-8"))
                    oldhdg = hdg
                    print(cstr)
                 if (speed != oldspeed):
                    cstr = "{v%4d}" % speed
                    tty.write(cstr.encode("utf-8"))
                    oldspeed = speed
                    print(cstr)
                 if (steer != oldsteer):
                    cstr = "{s%4d}" % steer
                    tty.write(cstr.encode("utf-8"))
                    oldsteer = steer
                    print(cstr)

                 epoch = time.time()


            flag = False
            cbuff = ""
            #end if flag
        # end main loop ========================

finally:
    robot.motor(0,0)
    print("Halted")
#    robot.deinit()