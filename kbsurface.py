#robot driver from keyboard lines 181204
import time
import motor_driver

robot = motor_driver.motor_driver()

steer = 0			# degrees clockwise
speed = 0			# percentage
left_limit = -35
right_limit = 35

#main loop
while True:
    key = input("...")
    if key != "":							#repeat?
        cmd = key
    print("cmd="+cmd)
    
    if cmd == 'j':
        print('left')
        if steer > left_limit:
            steer -= 1
            robot.motor(speed, steer)

    elif cmd == 'J':
        print('LEFT')
        if steer > left_limit - 3:
            steer -= 3
            robot.motor(speed, steer)

    elif cmd == 'k':
        print('right')
        if steer < right_limit:
            steer += 1
            robot.motor(speed, steer)
    
    elif cmd == 'K':
        print('RIGHT')
        if steer < right_limit + 3:
            steer += 3
            robot.motor(speed, steer)
    
    elif cmd == ' ':
        print('stop')
        steer = 0
        speed = 0
        robot.stop_all()

    elif cmd == 'f':
        if speed < 90:
            speed += 10
            robot.motor(speed, steer)

    elif cmd == 'r':
        if speed > -90:
            speed -= 10
            robot.motor(speed, steer)
	
    elif cmd == '/':
            steer = 0
            robot.motor(speed, steer)

    elif cmd == 'q' or cmd == 'Q':
        break

robot.stop_all()   
