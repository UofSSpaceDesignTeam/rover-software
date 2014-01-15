#!/usr/bin/python
import socket
import commands
import os
import time
import thread
import pygame, random
from pygame.locals import *

command = "nc 192.168.1.103 3001 -w 1 < request; nc -lu -p 3001 | mplayer -geometry 50%:10 -x 850 -y 450 -nosound -hardframedrop -noautosub -fps 35 -demuxer h264es -nocache -"
screen = pygame.display.set_mode((0,0),pygame.RESIZABLE)
#screen = pygame.display.set_mode([1000,1000])

pygame.init()

def connect():
	os.system(command)

#
# This is where the main loop starts
#

thread.start_new_thread(connect,())

screen.fill([255,255,255])
 
mainloop, x, y, color, fontsize, delta, fps =  True, 25 , 0, (32,32,32), 35, 1, 30
 
Clock = pygame.time.Clock()
 
while mainloop:
    tickFPS = Clock.tick(fps)
    pygame.display.set_caption("Press Esc to quit. FPS: %.2f" % (Clock.get_fps()))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            mainloop = False # Be IDLE friendly!
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                mainloop = False # Be IDLE friendly!
    pygame.display.update()
 
pygame.quit()
