#!/usr/bin/python
import socket
import commands
import os
import signal
import time
import subprocess
import socket
from threading import Thread
from Textbox import *
from Button import *
import pygame, random
from pygame.locals import *


screen = pygame.display.set_mode((0,0),pygame.RESIZABLE)
#cmd = ['nc', '-lu', '-p', 3001]|['mplayer', '-geometry', '50%:30', '-x', 850, '-y', 450, '-nosound', '-nolirc', '-noborder', '-hardframedrop', '-noautosub', '-fps', 35, '-demuxer', 'h264es', '-nocache']
#screen = pygame.display.set_mode([1000,1000])

throttleDeadzone = 0.02
throttleScale = 15

steeringDeadzone = 0.1
steeringScale = 25

panDeadzone = 0.1
panScale = 40

tiltDeadzone = 0.1
tiltScale = 30

rover = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
roverIP = ("192.168.1.103",3000)

pygame.init()
pygame.joystick.init()
#try:
 #       controller = pygame.joystick.Joystick(0)
#except:
#        print("XBox controller not found!")
#        time.sleep(3)
#        quit()
#print("Using: " + controller.get_name())

def controlThread():
	try:
		while(True):
			(throttle, steering, pan, tilt) = getinput()
			rover.send("#CM" + chr(throttle) + chr(steering) + chr(pan) + chr(tilt))
			time.sleep(0.05)

	except:
		print("closing connection")
		rover.shutdown(socket.SHUT_RDWR)
		rover.close()
		time.sleep(3)
		raise


def controlConnect():
        print("Connecting to " + roverIP[0] + ":" + str(roverIP[1]))
        while(True):
                try:
                        rover.connect(roverIP)
                        break
                except socket.error:
                        print("Retrying...")
                        time.sleep(3)

        print("Connection Established")

def getinput():
        pygame.event.pump()
        
        throttle = controller.get_axis(2)
        if(throttle < throttleDeadzone and throttle > -throttleDeadzone):
                throttle = 0
        elif(throttle > throttleDeadzone):
                throttle -= throttleDeadzone
        else:
                throttle += throttleDeadzone         
        throttle *= throttleScale
        throttle = int(throttle) + 90
        if(throttle > 255):
                throttle = 255
        if(throttle < 0):
                throttle = 0
                
        steering = controller.get_axis(0)
        if(steering < steeringDeadzone and steering > -steeringDeadzone):
                steering = 0
        elif(steering > steeringDeadzone):
                steering -= steeringDeadzone
        else:
                steering += steeringDeadzone       
        steering *= steeringScale
        steering = int(steering) + 90
        if(steering > 255):
                steering = 255
        if(steering < 0):
                steering = 0

        pan = controller.get_axis(4)
        if(pan < panDeadzone and pan > -panDeadzone):
                pan = 0
        elif(pan > panDeadzone):
                pan -= panDeadzone
        else:
                pan += panDeadzone       
        pan *= panScale
        pan = int(pan) + 90
        if(pan > 255):
                pan = 255
        if(pan < 0):
                pan = 0

        tilt = -1.0 * controller.get_axis(3)
        if(tilt < tiltDeadzone and tilt > -tiltDeadzone):
                tilt = 0
        elif(tilt > tiltDeadzone):
                tilt -= tiltDeadzone
        else:
                tilt += tiltDeadzone       
        tilt *= tiltScale
        tilt = int(tilt) + 90
        if(tilt > 255):
                tilt = 255
        if(tilt < 0):
                tilt = 0
                
        return(throttle, steering, pan, tilt)

def camConnect(port , x, y, frameps, ip):
  global p  
  os.system("nc -v "+ str(ip) +" -z "+ str(port))
  command = "nc -lu -p "+ str(port) +" | mplayer -ontop -geometry 50%:50 -x " + str(x) + " -y " + str(y) + " -nosound -nolirc -noborder -hardframedrop -noautosub -fps " + str(frameps) + " -demuxer h264es -nocache -"
  p = subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
#
# This is where the main loop starts
#

#camera buttons
camera1Btn = Button('Camera 1')
camera2Btn = Button('Camera 2')
disconnectBtn = Button('Disconect')

#control Buttons
armBtn = Button('Arm')
moveBtn = Button('Move')

#experiment buttons
startExperimentBtn = Button('Start Experiment')
tableBtn = Button('Create Table')

#Boxes
cameraBox = Button('Camera Box')
experimentBox = Button('Experiment')
experimentBtnBox = Button('Experiment Controls')
innerBox = Button('USST')
cameraBtnBox = Button('Cameras')
controlBox = Button('Controls')

#TextBoxes

#xbox 360 controls
#controller.init()
#controlConnect()
#thread = Thread(target = controlThread)
#thread.start()

p = 0
global on
screen.fill([0,0,0])
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
   ip1 = '192.168.1.103'
   ip2 = '192.168.1.104'
   mouse = pygame.mouse.get_pos()
   pygame.display.set_caption("Press Esc to quit. FPS: %.2f" % (Clock.get_fps()))
   for event in pygame.event.get():
       if event.type == pygame.QUIT:
           mainloop = False # Be IDLE friendly!
       elif event.type == pygame.KEYDOWN:
           if event.key == pygame.K_ESCAPE:
               mainloop = False # Be IDLE friendly!
       elif event.type == pygame.MOUSEBUTTONDOWN:
           if camera1Btn.obj.collidepoint(mouse):
	       if not on:
		   camConnect(port, x, y, frameps, ip1)
		   on = True
	       elif on:
		   p.stdout.flush()
		   p.stdout.close()
		   camConnect(port, x, y, frameps, ip1)
	   elif camera2Btn.obj.collidepoint(mouse):
	       if not on:
		   camConnect(port, x, y, frameps, ip2)
		   on = True
	       elif on:
		   p.stdout.flush()
		   p.stdout.close()
		   camConnect(port, x, y, frameps, ip2)	      
           elif disconnectBtn.obj.collidepoint(mouse):
	       if on:
	           p.stdout.flush()
	           p.stdout.close()
	           on = False
	           
	   elif armBtn.obj.collidepoint(mouse):
	       armBtn.selected = True
	       moveBtn.selected = False
	   
	   elif moveBtn.obj.collidepoint(mouse):
	       armBtn.selected = False
               moveBtn.selected = True
 
   #Boxes
   cameraBtnBox.default_color = (100,100,100)
   cameraBtnBox.draw(screen, mouse, (0,0,150,150), (41,2))
   controlBox.default_color = (100,100,100)
   controlBox.draw(screen, mouse, (0,180, 240, 50), (90,182))
   cameraBox.font_color = (255,255,255)
   cameraBox.default_color = (100, 100, 100)
   cameraBox.draw(screen, mouse, (400,0,880,415),(780,420))
   experimentBox.font_color = (255,255,255)
   experimentBox.default_color = (100,100,100)
   experimentBox.draw(screen, mouse, (1100,600,500,300),(1300,585))
   innerBox.font_size = 500
   innerBox.font_color = (255,255,255)
   innerBox.default_color = (255,255,255)
   innerBox.draw(screen, mouse, (1102,602,496,296),(30,450))
   experimentBtnBox.default_color = (100,100,100)
   experimentBtnBox.draw(screen, mouse, (1160, 905,375,50), (1290,907))
     
  #camera Buttons           
   camera1Btn.default_color = (0, 255, 0)
   camera1Btn.draw(screen, mouse, (20,20,100,20), (40,22))
   camera2Btn.default_color = (0,255,0)
   camera2Btn.draw(screen, mouse, (20,50,100,20), (40,52))
   disconnectBtn.default_color = (255, 0, 0)
   disconnectBtn.draw(screen, mouse, (20,100,100,20), (35,102))
   
   #control Buttons
   armBtn.default_color = (0,0,255)
   armBtn.selected_color = (0,255,0)
   armBtn.draw(screen, mouse,  (10, 200, 100, 20), (45,202))
   moveBtn.selected_color = (0,255,0)
   moveBtn.default_color = (0,0,255)
   moveBtn.draw(screen, mouse, (120, 200, 100, 20), (150,202))
   
   #Experiment Buttons
   startExperimentBtn.default_color = (0,255,0)
   startExperimentBtn.draw(screen, mouse, (1180 ,925,135,20), (1190,927))
   tableBtn.default_color = (0,255,0)
   tableBtn.draw(screen,mouse,(1380, 925, 135, 20),(1400,927))

   pygame.display.update()
 
pygame.quit()
