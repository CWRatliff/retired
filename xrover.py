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

robot = motor_driver.motor_driver()

while True:
    chr = int(bus.read_byte(addr))
    if (chr >= 65) and (chr <= 90):
        print(chr)

        if chr == 81:			#Q
            sys.exit()
        if chr == 70:
             if speed <= 90:
                speed += (10 * accel)
                accel = 1
                robot.motor(speed, steer)

        elif chr == 66:                 #B
            if speed >= -90:
                speed -= (10 * accel)
                accel = 1
                robot.motor(speed, steer)

        elif chr == 76:                 #L
            if steer > left_limit:
                steer -= (2 * accel)
                accel = 1
                robot.motor(speed, steer)

        elif chr == 82:                 #R
            if steer < right_limit:
                steer += (2 * accel)
                accel = 1
                robot.motor(speed, steer)

        elif chr == 83:                 #S
            speed = 0
            robot.motor(speed, steer)

        elif chr == 90:                 #Z
            robot.motor_speed()
            '''
            dt = 2
            if steer > 0:
                dt = -2
                while abs(steer) > 2:
                    steer += dt
                    robot.motor(speed, steer)
                    time.sleep(0.1)
            steer = 0
            robot.motor(speed, steer)
            '''

        elif chr == 88:                 #X
           robot.stop_all()
           exit()
        print("Motor speed, steer "+str(speed)+", "+str(steer))
        
        bus.write_byte(addr, ord('{'))
        bus.write_byte(addr, ord('v'))
        spd = hex(speed & 0xff)
        bus.write_byte(addr, ord(spd[2:3]))
        if len(spd) > 3:
            bus.write_byte(addr, ord(spd[3:4]))
        bus.write_byte(addr, ord(','))
        sta = hex(steer & 0xff)
        bus.write_byte(addr, ord(sta[2:3]))
        if len(sta) > 3:
            bus.write_byte(addr, ord(sta[3:4]))
        bus.write_byte(addr, ord('}'))
