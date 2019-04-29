#PlayStation interface via Xbee radios
import sys
import time
import smbus
import motor_driver


addr = 0x08
bus = smbus.SMBus(1)
steer = 0
speed = 0
accel = 1
left_limit = -35
right_limit = 35
epoch = time.time()

robot = motor_driver.motor_driver()

#while True:                             #purge xbee
#    chr = int(bus.read_byte(addr))
#    if (chr == 0):
#        break

while True:
    chr = int(bus.read_byte(addr))
	if (chr == 123):					# '{' -> start of message
		wstr = ""
		while True:
			chr = int(bus.read_byte(addr))
			wstr += chr;
			if (chr == 125):			# '}' -> end of message
				continue
			if (chr == 123):			# restart
				wstr = ""
		chr = wstr[0]					# grab lead character

    if (chr == 42):
#       print("ping")
       epoch = time.time()
	   continue
       
    if (chr >= 65) and (chr <= 90):

        if chr == 81:					#Q - quit
            sys.exit()

        if chr == 70:					#F - forward
             if speed <= 90:
                speed += (10 * accel)
                accel = 1
                robot.motor(speed, steer)

        elif chr == 66:                 #B - backup
            if speed >= -90:
                speed -= (10 * accel)
                accel = 1
                robot.motor(speed, steer)

        elif chr == 76:                 #L - left 2 deg
            if steer > left_limit:
                steer -= (2 * accel)
                accel = 1
                robot.motor(speed, steer)

        elif chr == 82:                 #R - right 2 deg
            if steer < right_limit:
                steer += (2 * accel)
                accel = 1
                robot.motor(speed, steer)

        elif chr == 71:                #GEE - hard a starboard
            dt = 1
            if steer < (right_limit - 1):
                while (steer < right_limit):
                    steer += dt
                    robot.motor(speed, steer)
#                    time.sleep(0.05)
            steer = right_limit
            robot.motor(speed, steer)

        elif chr == 72:                #HAW - hard a port
            dt = 1
            if steer > (left_limit + 1):
                while (steer > left_limit):
                    steer -= dt
                    robot.motor(speed, steer)
#                    time.sleep(0.05)
            steer = left_limit
            robot.motor(speed, steer)

        elif chr == 83:                 #S - stop drive motors
            speed = 0
            robot.motor(speed, steer)

        elif chr == 90:                 #Z - steer straight ahead
            dt = 1
            if steer > 0:
                dt = -1
                while abs(steer) > 1:
                    steer += dt
                    robot.motor(speed, steer)
#                    time.sleep(0.05)
            steer = 0
            robot.motor(speed, steer)

        elif chr == 88:                #X exit Select button
            robot.stop_all()
            speed = 0
            exit()
            
		elif chr == 79:					#O - orientation esp hdg from arduino
			eos = wstr.find('}')
			hdg = int(wstr[1:eos])

        #end if (chr >= 65) and (chr <= 90) =======================

    msg = "{v"+str(speed)+","+str(steer)+","+str(hdg)+"}""
	print(msg)
	i = 0
	while True:
		xchr = msg[i]
		bus.write_byte(addr, ord(xchr))
		if (xchr == '}'):
			continue

    epoch = time.time()

    if (time.time() > (epoch + 1.1)):
#        print(time.time(),5)
#        print(epoch,5)
        robot.stop_all()
        speed = 0;
        
    # end loop ========================
robot.deinit()