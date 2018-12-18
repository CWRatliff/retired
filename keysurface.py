#keystroke oriented commands, no screen output
import pygame
import sys
from pygame.locals import *

import time
import motor_driver

pygame.init()
screen = pygame.display.set_mode((600, 400))
pygame.display.set_caption('Rover')

steer = 0
speed = 0
accel = 1
left_limit = -35
right_limit = 35

robot = motor_driver.motor_driver()

while True:
    for event in pygame.event.get():
        if event.type == QUIT:			#X clicked in right corner
            sys.exit()
        if event.type == KEYDOWN:
#            print('key = '+str(event.key))
            if (event.key == K_RCTRL) or (event.key == K_RSHIFT):
                accel = 2
            elif event.key == K_UP:
                if speed <= 90:
                    speed += (10 * accel)
                    accel = 1
                    robot.motor(speed, steer)

            elif event.key == K_DOWN:
                if speed >= -90:
                    speed -= (10 * accel)
                    accel = 1
                    robot.motor(speed, steer)

            elif event.key == K_RIGHT:
                if steer > left_limit:
                    steer += (2 * accel)
                    accel = 1
#                    print("steer = "+str(steer))
                    robot.motor(speed, steer)

            elif event.key == K_LEFT:
                if steer < right_limit:
                    steer -= (2 * accel)
                    accel = 1
                    robot.motor(speed, steer)

            elif event.key == K_SPACE:
                speed = 0
                robot.motor(speed, steer)

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
				
