#!/usr/bin/python

# The main program for controlling the rover
# Added by Dylan

	# dependency list

import pygame
from Button import Button
from Box import Box
import Controller
from Communication import Communication
import socket
import commands
import os
import signal
import time
from datetime import date
import subprocess
from threading import Thread


	# global constants

useController = False

driveControlIP = "192.168.1.103"
driveControlPort = 3000

armControlIP = "192.168.1.104"
armControlPort = 3000

cameraFrameRate = "35"
cameraPort = "3001"

commandCameraStart = "#CS"
commandTakePicture = ""
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
	global buttonList
	buttonList = []
	
	camera1Button = Button(camConnect, (1), "Camera 1", 20, colorBlack, (12, 20, 100, 20), colorBlue, colorYellow)
	camera2Button = Button(camConnect, (2), "Camera 2", 20, colorBlack, (12, 50, 100, 20), colorBlue, colorYellow)
	camera3Button = Button(camConnect, (3), "Camera 3", 20, colorBlack, (12, 80, 100, 20), colorBlue, colorYellow)
	camera4Button = Button(camConnect, (4), "Camera 4", 20, colorBlack, (12, 110, 100, 20), colorBlue, colorYellow)
	cameraStopButton = Button(camConnect, (0), "None", 20, colorBlack, (12, 140, 100, 20), colorBlue, colorYellow)
	cameraStopButton.selected = True
	buttonList.append(camera1Button)	# 0
	buttonList.append(camera2Button)	# 1
	buttonList.append(camera3Button)	# 2
	buttonList.append(camera4Button)	# 3
	buttonList.append(cameraStopButton)	# 4
	
	
	moveButton = Button(setMode, ("drive"), "Drive", 20, colorRed, (12, 205, 100, 20), colorBlue, colorYellow)
	armButton = Button(setMode, ("arm"), "Arm", 20, colorRed, (12, 235, 100, 20), colorBlue, colorYellow)
	moveButton.selected = True
	buttonList.append(moveButton)	# 5
	buttonList.append(armButton)	# 6
	
	pictureButton = Button(takePicture, None, "Take Picture", 20, colorWhite, (475, 425, 200, 30), colorBlue, colorYellow)
	buttonList.append(pictureButton)	# 7
	
	#runExperimentButton = Button("Run", 20, colorBlack, (1180, 925, 135, 20), colorBlue, colorYellow)
	#createTableButton = Button("Plot Data", 20, colorBlack, (1380, 925, 135, 20), colorGray, colorYellow)
	#buttonList.append(runExperimentButton)
	#buttonList.append(createTableButton)


def createBoxes():
	global boxList
	boxList = []
	
	cameraBox = Box("Video Area", 40, colorWhite, (140, 0, 880, 415), (500, 180), colorGray)
	cameraButtonBox = Box("Camera Feeds", 20, colorWhite, (0, 0, 125, 170), (17, 2), colorGray)
	boxList.append(cameraBox)
	boxList.append(cameraButtonBox)
	
	controlBox = Box("Control Modes", 20, colorWhite, (0, 185, 125, 80), (16, 188), colorGray)
	boxList.append(controlBox)
	
	experimentBox = Box("Data Plot Area", 40, colorWhite, (1040, 420, 460, 400), (1200, 500), colorGray)
	experimentButtonBox = Box("Experiment", 20, colorWhite, (1160, 905, 375, 50), (1290, 907), colorGray)
	boxList.append(experimentBox)
	boxList.append(experimentButtonBox)
	
	fillerBox = Box("USST", 500, colorGreen, (1102, 602, 496, 296), (30, 450), colorWhite)
	boxList.append(fillerBox)


def drawButtons():
	for i in buttonList:
		i.draw(screen)


def drawBoxes():
	for i in boxList:
		i.draw(screen)


def setMode(mode):	# button-based
	global controlMode
	for modeButton in buttonList[5:7]:
		modeButton.selected = False
	if(mode == "drive"):
		controlMode = "drive"
		buttonList[5].selected = True
	if(mode == "arm"):
		controlMode = "arm"
		buttonList[6].selected = True
	drawButtons()

def getInput():
	global axes, buttons, dPad
	axes = Controller.getAxes()
	buttons = Controller.getButtons()
	dPad = Controller.getDPad()
	
def takePicture(fakeArg):	# button-based
	if(buttonList[4].selected):
		return
		
	try:
		p.stdout.flush
		p.stdout.close
	except:
		pass
	
	if(buttonList[0].selected):
		driveControlConnection.send(commandTakePicture)
		driveControlConnection.retrieveFile("picture.jpg")
		camConnect(1)
	elif(buttonList[1].selected):
		armControlConnection.send(commandTakePicture)
		armControlConnection.retrieveFile("picture.jpg")
		camConnect(2)
	elif(buttonList[2].selected):
		camConnect(3)
	elif(buttonList[3].selected):
		camConnect(4)


def camConnect(cameraNumber): # button-based
	global p
	try:
		p.stdout.flush
		p.stdout.close
	except:
		pass
	
	for cameraButton in buttonList[0:5]:
		cameraButton.selected = False
	
	if(cameraNumber == 0):
		buttonList[4].selected = True
	elif(cameraNumber == 1):
		driveControlConnection.send(commandCameraStart)
		buttonList[0].selected = True
	elif(cameraNumber == 2):
		armControlConnection.send(commandCameraStart)
		buttonList[1].selected = True
	elif(cameraNumber == 3):
		buttonList[2].selected = True
	elif(cameraNumber == 4):
		buttonList[3].selected = True
	
	drawBoxes()
	drawButtons()
	
	# command = ("stdbuf -i50 -o50 nc -l -p "+ cameraPort + " | stdbuf -i50 -o50 mplayer -x 850 -y 400" +
			# " -nosound + -hardframedrop -noautosub -fps " + cameraFrameRate + " -ontop" +
			# " -geometry 50%:50 -demuxer h264es -nocache -")
	# p = subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE,
		# stdout=subprocess.PIPE, stderr=None)


	# main execution
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (25,30)
pygame.init()
Clock = pygame.time.Clock()
screen = pygame.display.set_mode((1500, 800))
if(useController):
	Controller.init()
driveControlConnection = Communication(driveControlIP, driveControlPort)
armControlConnection = Communication(armControlIP, armControlPort)
createButtons()
createBoxes()
drawBoxes()
drawButtons()
pygame.display.update()

mainloop = True
 
while mainloop:
	escapecmd = 'QUIT\n'
	mouse = pygame.mouse.get_pos()
	if(useController):
		getInput()
	Clock.tick(30)
	pygame.display.set_caption("Press Esc to quit. FPS: " + str(round(Clock.get_fps(), 2)))
	
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			mainloop = False
		elif event.type == pygame.KEYDOWN:
			if event.key == pygame.K_ESCAPE:
				mainloop = False
		elif event.type == pygame.MOUSEBUTTONDOWN:
			for button in buttonList:
				if(button.obj.collidepoint(mouse)):
					button.press()
	
	pygame.display.update()


	# end of execution cleanup

pygame.quit()
