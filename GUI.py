#!/usr/bin/python

# The main program for controlling the rover
# Added by Dylan

	# dependency list

import pygame
import Button
import Box
import Controller
import Communication
import socket
import commands
import os
import signal
import time
import subprocess
from threading import Thread


	# global constants

driveControlIP = "192.168.1.103"
driveControlPort = 3000

armControlIP = "192.168.1.104"
armControlPort = 3000

colorWhite = (255, 255, 255)
colorGray = (125, 125, 125)
colorBlack = (0, 0, 0)
colorRed = (240, 0, 0)
colorGreen = (0, 240, 0)
colorBlue = (0, 0, 240)
colorYellow = (250, 250, 0)


	# function definitions

def createButtons():
	global buttonList = []
	
	camera1Button = Button("Camera 1", 20, colorBlack, (20, 20, 100, 20), colorBlue, colorYellow)
	camera2Button = Button("Camera 2", 20, colorBlack, (20, 50, 100, 20), colorBlue, colorYellow)
	camera3Button = Button("Camera 3", 20, colorBlack, (20, 50, 100, 20), colorGray, colorYellow)
	camera4Button = Button("Camera 4", 20, colorBlack, (20, 50, 100, 20), colorGray, colorYellow)
	cameraStopButton = Button("None", 20, colorBlack, (20, 100, 100, 20), colorBlue, colorYellow)
	cameraStopButton.selected = True
	buttonList.append(camera1Button)
	buttonList.append(camera2Button)
	buttonList.append(camera3Button)
	buttonList.append(camera4Button)
	buttonList.append(cameraStopButton)
	
	armButton = Button("Arm", 30, colorRed, (10, 200, 100, 20), colorBlue, colorYellow)
	moveButton = Button("Drive", 30, colorRed, (120, 200, 100, 20), colorBlue, colorYellow)
	moveButton.selected = True
	buttonList.append(armButton)
	buttonList.append(moveButton)
	
	runExperimentButton = Button("Run", 20, colorBlack, (1180, 925, 135, 20), colorBlue, colorYellow)
	createTableButton = Button("Plot Data", 20, colorBlack, (1380, 925, 135, 20), colorGray, colorYellow)
	buttonList.append(runExperimentButton)
	buttonList.append(createTableButton)


def createBoxes():
	global boxList = []
	
	cameraBox = Box("Video Area", 20, colorWhite, (400, 0, 880, 415), (780, 420), colorGray)
	cameraButtonBox = Box("Camera Feeds", 20, colorWhite, (0, 0, 150, 150), (41, 2), colorGray)
	boxList.append(cameraBox)
	boxList.append(cameraButtonBox)
	
	experimentBox = Box("Data Plot Area", 20, colorWhite, (1100, 600, 500, 300), (1300, 585), colorGray)
	experimentButtonBox = Box("Experiment", 20, colorWhite, (1160, 905, 375, 50), (1290, 907), colorGray)
	boxList.append(experimentBox)
	boxList.append(experimentButtonBox)
	
	fillerBox = Box("USST", 500, colorGreen, (1102, 602, 496, 296), (30, 450), colorWhite)
	boxList.append(fillerBox)
	
	controlBox = Box("Controls", 20, colorWhite, (0, 180, 240, 50), (90, 182), colorGray)
	boxList.append(controlBox)


def drawButtons():
	for i in buttonList
		i.draw(screen)


def drawBoxes():
	for i in boxList
		i.draw(screen)


def controlThread():
	try:
		while(True):
			(throttle, steering, pan, tilt) = getinput()
			moveControl.move(chr(throttle) + chr(steering) + chr(pan) + chr(tilt))

	except:
		print("closing connection")
		moveControl.end(socket.SHUT_RDWR)
		raise


def controlConnect():
	xboxControl.link(controlState)


def getinput():
	global axes, buttons, dPad
	axes = Controller.getAxes()
	buttons = Controller.getButtons()
	dPad = Controller.getDPad()


def camConnect(port , x, y, frameps, ip):
  global p
  xboxControl.startCamera()
  command = "stdbuf -i50 -o50 nc -l -p 3001 | stdbuf -i50 -o50 mplayer -nosound -hardframedrop -noautosub -fps 35 -demuxer h264es -nocache -"
  p = subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
  os.system("echo request | nc -c 1 -v "+ str(ip) + " " + str(port))
  #command = "nc -l -p "+ str(port) +" | mplayer -ontop -geometry 50%:50 -x " + str(x) + " -y " + str(y) + " -nosound -nolirc -noborder -framedrop -noautosub -fps " + str(frameps) + " -demuxer h264es -nocache -"


	# main execution

pygame.init()
screen = pygame.display.set_mode(1600, 900)
Controller.setup()
driveControlConnection = Communication(driveControlIP, driveControlPort)
armControlConnection = Communication(armControlIP, armControlPort)


#Control setup
#cThr = Thread(target = controlConnect)
#cThr.start()


#thread = Thread(target = controlThread)
#thread.start()

 
mainloop, x, y, color, fontsize, delta, fps =  True, 25 , 0, (32,32,32), 35, 1, 30
 
Clock = pygame.time.Clock()
 
while mainloop:
   escapecmd = 'QUIT\n'
   frameps = 35
   x = 850
   y = 400
   port = 3001
   ip1 = '192.168.1.103'
   ip2 = '192.168.1.104'
   tickFPS = Clock.tick(fps)
   mouse = pygame.mouse.get_pos()
   pygame.display.set_caption("Press Esc to quit. FPS: %.2f" % (Clock.get_fps()))
   for event in pygame.event.get():
       if event.type == pygame.QUIT:
           mainloop = False # Be IDLE friendly!
       elif event.type == pygame.KEYDOWN:
           if event.key == pygame.K_ESCAPE:
               mainloop = False # Be IDLE friendly!
       elif event.type == pygame.MOUSEBUTTONDOWN:
           if camera1Button.obj.collidepoint(mouse):
	       if not on:
		   camConnect(port, x, y, frameps, ip1)
		   on = True
	       elif on:
		   print 
		   p.stdout.flush()
		   p.stdout.close()
		   camConnect(port, x, y, frameps, ip1)
	   elif camera2Button.obj.collidepoint(mouse):
	       if not on:
		   camConnect(port, x, y, frameps, ip2)
		   on = True
	       elif on:
		   p.stdout.flush()
		   p.stdout.close()
		   camConnect(port, x, y, frameps, ip2)	      
           elif disconnectButton.obj.collidepoint(mouse):
	       if on:
	           p.stdout.flush()
	           p.stdout.close()
	           on = False
	           
	   elif armButton.obj.collidepoint(mouse):
	       armButton.selected = True
	       moveButton.selected = False
	       moveButton.draw(screen, mouse, (120, 200, 100, 20), (150,202))
	       armButton.draw(screen, mouse,  (10, 200, 100, 20), (45,202))
	       controlState = 'arm'
	       #xboxControl.disconnectMove()
	       #xboxControl.link(controlState)
	   
	   elif moveButton.obj.collidepoint(mouse):
	       armButton.selected = False
               moveButton.selected = True
               moveButton.draw(screen, mouse, (120, 200, 100, 20), (150,202))
	       armButton.draw(screen, mouse,  (10, 200, 100, 20), (45,202))
               controlState = 'move'
               #xboxControl.disconnectArm()
	       #xboxControl.link(controlState)



   pygame.display.update()
 
pygame.quit()
