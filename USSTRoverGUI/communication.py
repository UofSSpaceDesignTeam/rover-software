#!/usr/bin/python
import socket
import commands
import os
import signal
import time
import subprocess
from Textbox import *
from Button import *
import pygame, random
from pygame.locals import *


screen = pygame.display.set_mode((0,0),pygame.RESIZABLE)
#cmd = ['nc', '-lu', '-p', 3001]|['mplayer', '-geometry', '50%:30', '-x', 850, '-y', 450, '-nosound', '-nolirc', '-noborder', '-hardframedrop', '-noautosub', '-fps', 35, '-demuxer', 'h264es', '-nocache']
#screen = pygame.display.set_mode([1000,1000])

pygame.init()

def connect(port , x, y, frameps):
  global p  
  os.system("nc -v 192.168.1.110 -z "+ str(port))
  command = "nc -lu -p "+ str(port) +" | mplayer -ontop -geometry 50%:50 -x " + str(x) + " -y " + str(y) + " -nosound -nolirc -noborder -hardframedrop -noautosub -fps " + str(frameps) + " -demuxer h264es -nocache -"
  p = subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
#
# This is where the main loop starts
#

connectBtn = Button('Connect')
disconnectBtn = Button('Disconect')
#btn3 = Button('Button 3')
p = 0
global on
screen.fill([255,255,255])
on = False
 
mainloop, x, y, color, fontsize, delta, fps =  True, 25 , 0, (32,32,32), 35, 1, 30
 
Clock = pygame.time.Clock()
 
while mainloop:
   tickFPS = Clock.tick(fps)
   escapecmd = 'QUIT\n'
   frameps = 35
   x = 850
   y = 400
   port = 3001
   portTextBox = TextBox(pygame.Rect(100,300, 300,25), 1)
   mouse = pygame.mouse.get_pos()
   pygame.display.set_caption("Press Esc to quit. FPS: %.2f" % (Clock.get_fps()))
   for event in pygame.event.get():
       if event.type == pygame.QUIT:
           mainloop = False # Be IDLE friendly!
       elif event.type == pygame.KEYDOWN:
           if event.key == pygame.K_ESCAPE:
               mainloop = False # Be IDLE friendly!
       elif event.type == pygame.MOUSEBUTTONDOWN:
           if connectBtn.obj.collidepoint(mouse):
	       print on
	       if not on:
		   connect(port, x, y, frameps)
		   on = True
		   print on
	      
           elif disconnectBtn.obj.collidepoint(mouse):
	       print on
	       if on:
	           p.stdout.flush()
	           p.stdout.close()
	           on = False
	           print on  
	   elif portTextBox.rect.collidepoint(pygame.mouse.get_pos()):
               portTextBox.selected = True
           else:
              portTextBox.selected = False

          # elif btn3.obj.collidepoint(mouse):
           #   print('button 3 clicked')
              
   connectBtn.default_color = (0, 255, 0)
   connectBtn.draw(screen, mouse, (20,20,100,20), (40,22))
   disconnectBtn.default_color = (255, 0, 0)
   disconnectBtn.draw(screen, mouse, (20,50,100,20), (35,52))
   portTextBox.update(screen)
   #btn3.draw(screen, mouse, (100,160,100,20), (125,163))
   pygame.display.update()
 
pygame.quit()
