#!/usr/bin/python
import socket
import commands
import os
import signal
import time
import subprocess
import Button
import pygame, random
from pygame.locals import *


screen = pygame.display.set_mode((0,0),pygame.RESIZABLE)
#cmd = ['nc', '-lu', '-p', 3001]|['mplayer', '-geometry', '50%:30', '-x', 850, '-y', 450, '-nosound', '-nolirc', '-noborder', '-hardframedrop', '-noautosub', '-fps', 35, '-demuxer', 'h264es', '-nocache']
#screen = pygame.display.set_mode([1000,1000])

pygame.init()

def connect(port , x, y, frameps):
  global p
  command = "nc -lu -p 3001 | mplayer -geometry 50%:30 -x " + str(x) + " -y " + str(y) + " -nosound -nolirc -noborder -hardframedrop -noautosub -fps " + str(frameps) + " -demuxer h264es -nocache -"
  p = subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
#	os.system(command)
#
# This is where the main loop starts
#

btn = Button.Button('Connect')
btn2 = Button.Button('Disconect')
btn3 = Button.Button('Button 3')
p = 0
screen.fill([255,255,255])
 
mainloop, x, y, color, fontsize, delta, fps =  True, 25 , 0, (32,32,32), 35, 1, 30
 
Clock = pygame.time.Clock()
 
while mainloop:
   tickFPS = Clock.tick(fps)
   escapecmd = 'QUIT\n'
   frameps = 35
   x = 830
   y = 450
   port = 3001
   mouse = pygame.mouse.get_pos()
   pygame.display.set_caption("Press Esc to quit. FPS: %.2f" % (Clock.get_fps()))
   for event in pygame.event.get():
       if event.type == pygame.QUIT:
           mainloop = False # Be IDLE friendly!
       elif event.type == pygame.KEYDOWN:
           if event.key == pygame.K_ESCAPE:
               mainloop = False # Be IDLE friendly!
       elif event.type == pygame.MOUSEBUTTONDOWN:
           if btn.obj.collidepoint(mouse):
              connect(port, x, y, frameps)
           elif btn2.obj.collidepoint(mouse):
              p.stdout.flush()
              p.stdout.close()

           elif btn3.obj.collidepoint(mouse):
              print('button 3 clicked')
              
   btn.draw(screen, mouse, (100,100,100,20), (125,103))
   btn2.draw(screen, mouse, (100,130,100,20), (125,133))
   btn3.draw(screen, mouse, (100,160,100,20), (125,163))
   pygame.display.update()
 
pygame.quit()
