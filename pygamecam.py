#from raspirobotboard import *
import pygame
import pygame.camera
import sys
from pygame.locals import *

import time
#from board import SCL, SDA
#mport busio

#from adafruit_pca9685 import PCA9685

#from adafruit_motor import servo
#i2c = busio.I2C(SCL, SDA)
#pca = PCA9685(i2c)
#pca.frequency = 50
#servoFL = servo.Servo(pca.channels[0],actuation_range=119,min_pulse=700,max_pulse=2300)
#servoFL.angle = 90

pygame.init()
pygame.camera.init()

screen = pygame.display.set_mode((640, 480))

pygame.display.set_caption('RaspiRobot')
#cam = pygame.camera.Camera("/dev/video0", (640, 480))
#cam.start()
#image = cam.get_image()

#camlist = pygame.camera.list_cameras()
#if camlist:
#    cam = pygame.camera.Camera(camlist[0], (640,480))
#pygame.mouse.set_visible(0)

from picamera import PiCamera
camera = PiCamera()
camera.preview_fullscreen = False
camera.preview_window=(0, 0, 640, 480)
camera.start_preview()
time.sleep(10)
camera.stop_preview()

steer = 90
speed = 0
left_limit = 60
right_limit = 120
GREEN = (0, 255, 0)

while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_UP:
                print("up")
                steer, speed = pygame.mouse.get_pos()
                pygame.display('s = '+str(steer))
                #rr.forward()
                #rr.set_led1(True)
                #rr.set_led2(True)
            elif event.key == K_DOWN:
                print('down')
                pygame.draw.line(screen, GREEN, [0, 0], [50, 30,], 5)
                pygame.display.flip()
                #rr.set_led1(True)
                #rr.set_led2(True)
                #rr.reverse()
            elif event.key == K_RIGHT:
                print('right')
                if steer > left_limit:
                    steer -= 1
  #                  servoFL.angle = steer
                #rr.set_led1(False)
                #rr.set_led2(True)
                #rr.right()
            elif event.key == K_LEFT:
                print('left')
                if steer < right_limit:
                    steer += 1
    #                servoFL.angle = steer
                #rr.set_led1(True)
                #rr.set_led2(False)
                #rr.left()
            elif event.key == K_SPACE:
                print('stop')

                #rr.stop()
                #rr.set_led1(False)
                #rr.set_led2(False)
