#web page commands
from bottle import route, run, template, request
import sys
import time
import motor_driver

IP_ADDRESS = '192.168.1.10'

steer = 0
speed = 0
left_limit = -35
right_limit = 35

robot = motor_driver.motor_driver()

@route('/')
def index():
    global steer
    global speed
    global left_limit
    global right_limit
    
    cmd = request.GET.get('command', '')
    if cmd == 'f':
        if speed <= 90:
            speed += 10
            robot.motor(speed, steer)

    elif cmd == 'b':
        if speed >= -90:
            speed -= 10
            robot.motor(speed, steer)

    elif cmd == 'r':
        if steer > left_limit:
            steer -= 2
#               print("steer = "+str(steer))
            robot.motor(speed, steer)

    elif cmd == 'l':
        if steer < right_limit:
            steer += 2
            robot.motor(speed, steer)


    elif cmd == 's':
        speed = 0
        robot.motor(speed, steer)
        
    else:
        print('cmd = '+cmd)
    return template('home.tpl')

try:
    run(host=IP_ADDRESS, port=80)
finally:
    print('finally')

''''
            elif event.key == K_RETURN:
                dt = 2
                if steer > 0:
                    dt = -2
                while abs(steer) > 2:
                    steer += dt
                    robot.motor(speed, steer)
                    time.sleep(0.1)
                steer = 0
                robot.motor(speed, steer)

            elif event.key == K_ESCAPE:
                robot.stop_all()
'''
