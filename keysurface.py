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
left_limit = -35
right_limit = 35

robot = motor_driver.motor_driver()

while True:
    for event in pygame.event.get():
        if event.type == QUIT:			#X clicked in right corner
            sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_UP:
                print('up')
                if speed <= 90:
                    speed += 10
                    robot.motor(speed, steer)

            elif event.key == K_DOWN:
                if speed >= -90:
                    speed -= 10
                    robot.motor(speed, steer)

            elif event.key == K_RIGHT:
                if steer > left_limit:
                    steer -= 1
                    print("steer = "+str(steer))
                    robot.motor(speed, steer)

            elif event.key == K_LEFT:
                if steer < right_limit:
                    steer += 1
                    robot.motor(speed, steer)

            elif event.key == (K_RIGHT and KMOD_CTRL) or event.key == (K_RIGHT and KMOD_SHIFT):
                if steer > left_limit + 3:
                    steer -= 3
                    robot.motor(speed, steer)

            elif event.key == (K_LEFT and KMOD_CTRL) or event.key == (K_LEFT and KMOD_SHIFT):
                if steer < right_limit - 3:
                    steer -= 3
                    robot.motor(speed, steer)

            elif event.key == K_SPACE:
                speed = 0
                robot.motor(speed, steer)

            elif event.key == K_RETURN:
                steer = 0
                robot.motor(speed, steer)

            elif event.key == K_ESCAPE:
                robot.stop_all()
				
