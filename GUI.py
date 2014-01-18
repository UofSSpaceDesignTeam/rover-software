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

cameraFramerate = "35"
cameraPort = "3001"

commandCameraStart = "#CS"
commandControllerData = ""

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
	
	camera1Button = Button((camConnect, 1), "Camera 1", 20, colorBlack, (20, 20, 100, 20), colorBlue, colorYellow)
	camera2Button = Button((camConnect, 2), "Camera 2", 20, colorBlack, (20, 50, 100, 20), colorBlue, colorYellow)
	camera3Button = Button((camConnect, 1), "Camera 3", 20, colorBlack, (20, 50, 100, 20), colorGray, colorYellow)
	camera4Button = Button((camConnect, 1), "Camera 4", 20, colorBlack, (20, 50, 100, 20), colorGray, colorYellow)
	cameraStopButton = Button((camConnect, 0), "None", 20, colorBlack, (20, 100, 100, 20), colorBlue, colorYellow)
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


def camConnect(cameraNumber):
	try:
		p.stdout.flush
		p.stdout.close
	except:
		pass
	for cameraButton in buttonList[0:4]
		cameraButton.selected = False
		cameraButton.draw()
	if(cameraNumber == 1)
		driveControlConnection.send(commandCameraStart)
	elif(cameraNumber == 2)
		armControlConnection.send(commandCameraStart)
	else
		return
	
	command = ("stdbuf -i50 -o50 nc -l -p "+ cameraPort + " | stdbuf -i50 -o50 mplayer -x 850 -y 400" +
			" -nosound + -hardframedrop -noautosub -fps " + cameraFrameRate + " -ontop" +
			" -geometry 50%:50 -demuxer h264es -nocache -")
	global p
	p = subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE,
		stdout=subprocess.PIPE, stderr=None)


	# main execution

pygame.init()
Clock = pygame.time.Clock()
tickFPS = Clock.tick(30)
screen = pygame.display.set_mode(1600, 900)
Controller.setup()
driveControlConnection = Communication(driveControlIP, driveControlPort)
armControlConnection = Communication(armControlIP, armControlPort)


#Control setup
#cThr = Thread(target = controlConnect)
#cThr.start()

#thread = Thread(target = controlThread)
#thread.start()

mainloop = True
 
while mainloop:
	#escapecmd = 'QUIT\n'
	pygame.event.pump()
	mouse = pygame.mouse.get_pos()
	pygame.display.set_caption("Press Esc to quit. FPS: " + str(round(Clock.get_fps(), 2))
	
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			mainloop = False
		elif event.type == pygame.KEYDOWN:
			if event.key == pygame.K_ESCAPE:
				mainloop = False
		elif event.type == pygame.MOUSEBUTTONDOWN:
			for button in buttonList:
				if(button.obj.collidepoint(mouse))
					button.press()
	
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

	
	# end of execution cleanup

pygame.quit()
