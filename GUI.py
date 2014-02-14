#!/usr/bin/python

# The main program for controlling the rover

	# dependency list

import sys
# disables generation of .pyc files
sys.dont_write_bytecode = True
	
import pygame
import Controller
from Button import Button
from Box import Box
from Indicator import Indicator
from DriveClient import DriveClient
from CameraClient import CameraClient
from ArmClient import ArmClient
from TextOutput import TextOutput
from VirtualRobot import VirtualRobot
import socket
import commands
import os
import signal
import time
from datetime import date
import subprocess


	# global constants

IPraspi1 = "192.168.1.103"
IPraspi2 = "192.168.1.104"
IPraspi3 = "192.168.1.105"
IPraspi4 = "192.168.1.106"

# netcat video on 3001
driveClientPort = 3002
armClientPort = 3003
cameraClientPort = 3000

colorWhite = (255, 255, 255)
colorGray = (100, 100, 100)
colorBlack = (0, 0, 0)
colorRed = (240, 0, 0)
colorGreen = (0, 240, 0)
colorDarkGreen = (0, 100, 0)
colorBlue = (0, 0, 240)
colorLightBlue = (100, 100, 250)
colorDarkBlue = (0, 0, 120)
colorYellow = (250, 250, 0)


	# function definitions

def createButtons():
	global buttonList
	buttonList = []
	camera1Button = Button(camConnect, (1), "Camera 1", 20, colorBlack, (12, 25, 100, 20), colorLightBlue, colorGreen)
	camera2Button = Button(camConnect, (2), "Camera 2", 20, colorBlack, (12, 55, 100, 20), colorLightBlue, colorGreen)
	camera3Button = Button(camConnect, (3), "Camera 3", 20, colorBlack, (12, 85, 100, 20), colorLightBlue, colorGreen)
	camera4Button = Button(camConnect, (4), "Camera 4", 20, colorBlack, (12, 115, 100, 20), colorLightBlue, colorGreen)
	cameraStopButton = Button(camDisconnect, (0), "None", 20, colorBlack, (12, 145, 100, 20), colorLightBlue, colorGreen)
	cameraStopButton.selected = True
	moveButton = Button(setMode, ("drive"), "Drive", 20, colorBlack, (12, 204, 100, 20), colorLightBlue, colorGreen)
	armButton = Button(setMode, ("arm"), "Arm", 20, colorBlack, (12, 234, 100, 20), colorLightBlue, colorGreen)
	moveButton.selected = True
	stopButton = Button(stopRover, None, "Stop", 22, colorYellow, (12, 294, 100, 20), colorRed, colorRed)
	pictureButton = Button(takePicture, None, "Picture", 20, colorBlack, (12, 324, 100, 20), colorLightBlue, colorYellow)
	runExperimentButton = Button(runExperiment, None, "Science!", 20, colorBlack, (12, 354, 100, 20), colorLightBlue, colorYellow)
	connectButton = Button(connectClients, None, "Connect All", 20, colorBlack, (1107, 180, 100, 20), colorLightBlue, colorYellow)
	quitButton = Button(quit, None, "Quit", 20, colorBlack, (12, 415, 100, 20), colorLightBlue, colorYellow)
	buttonList.append(camera1Button)	# 0
	buttonList.append(camera2Button)	# 1
	buttonList.append(camera3Button)	# 2
	buttonList.append(camera4Button)	# 3
	buttonList.append(cameraStopButton)	# 4
	buttonList.append(moveButton)	# 5
	buttonList.append(armButton)	# 6
	buttonList.append(stopButton)	# 7
	buttonList.append(pictureButton)	# 8
	buttonList.append(runExperimentButton)	# 9
	buttonList.append(connectButton)	# 10
	buttonList.append(quitButton)	# 11

def createBoxes():
	global boxList
	boxList = []
	cameraButtonBox = Box("Camera Feeds", 22, colorWhite, (0, 0, 125, 175), (11, 6), colorGray)
	controlBox = Box("Control Modes", 22, colorWhite, (0, 180, 125, 83), (9, 185), colorGray)
	actionBox = Box("Rover Actions", 22, colorWhite, (0, 268, 125, 115), (10, 274), colorGray)
	uiBox = Box("User Interface", 22, colorWhite, (0, 389, 125, 55), (12, 395), colorGray)
	connectionsBox = Box("Connections", 22, colorWhite, (1095, 0, 125, 235), (1110, 6), colorGray)
	boxList.append(cameraButtonBox)
	boxList.append(controlBox)
	boxList.append(actionBox)
	boxList.append(uiBox)
	boxList.append(connectionsBox)

def createIndicators():
	global indicatorList
	indicatorList = []
	camera1Indicator = Indicator(checkClient, cameraRaspi1, "Camera 1", colorWhite, (1108, 30), colorRed, colorGreen)
	camera2Indicator = Indicator(checkClient, cameraRaspi2, "Camera 2", colorWhite, (1108, 55), colorRed, colorGreen)
	camera3Indicator = Indicator(checkClient, cameraRaspi3, "Camera 3", colorWhite, (1108, 80), colorRed, colorGreen)
	camera4Indicator = Indicator(checkClient, cameraRaspi4, "Camera 4", colorWhite, (1108, 105), colorRed, colorGreen)
	driveIndicator = Indicator(checkClient, driveControl, "Drive System", colorWhite, (1108, 130), colorRed, colorGreen)
	armIndicator = Indicator(checkClient, armControl, "Arm System", colorWhite, (1108, 155), colorRed, colorGreen)
	controllerIndicator = Indicator(checkController, None, "Controller", colorWhite, (1108, 210), colorRed, colorGreen)
	indicatorList.append(camera1Indicator) #0
	indicatorList.append(camera2Indicator) #1
	indicatorList.append(camera3Indicator) #2
	indicatorList.append(camera4Indicator) #3
	indicatorList.append(driveIndicator) #4
	indicatorList.append(armIndicator) #5
	indicatorList.append(controllerIndicator) #6

def createRobot():
	global robot
	robot = virtualRobot(1110,500,100,150)

def connectConsole():
	global output
	global error
	output = TextOutput(15, colorGreen, (4, 542, 350, 158), 13, colorGray)
	error = TextOutput(15, colorRed, (348, 542, 350, 158), 13, colorGray)
	#connect stderr and stdout to output
	sys.stdout = output
	sys.stderr = error

def drawButtons():
	for i in buttonList:
		i.draw(screen)

def drawBoxes():
	for i in boxList:
		i.draw(screen)

def drawIndicators():
	for i in indicatorList:
		i.refresh()
		i.draw(screen)
	for i in range(0, 4):
		if buttonList[i].selected:
			if not indicatorList[i].active:
				camDisconnect(None)
				drawButtons()

def setMode(mode):	# button-based
	global controlMode
	for modeButton in buttonList[5:7]:
		modeButton.selected = False
	if mode == "drive":
		controlMode = "drive"
		buttonList[5].selected = True
	if(mode == "arm"):
		controlMode = "arm"
		buttonList[6].selected = True
	drawButtons()

def checkController(fakeArg): # button-based
	return Controller.isConnected

def getInput():
	global axes, buttons, dPad
	axes = Controller.getAxes()
	buttons = Controller.getButtons()
	dPad = Controller.getDPad()

def takePicture(fakeArg):	# button-based
	cameraNumber = 0
	for i in range(0, 4):
		if buttonList[i].selected:
			cameraNumber = i + 1
	if cameraNumber == 0:
		return
	buttonList[8].selected = True
	camDisconnect(None)
	drawButtons()
	screen.blit(cameraSplash, (130, 0))
	pygame.display.update()
	time.sleep(1)
	if cameraNumber == 1:
		cameraRaspi1.takePicture()
		time.sleep(2.5)
		camConnect(1)
	elif cameraNumber == 2:
		cameraRaspi2.takePicture()
		time.sleep(2.5)
		camConnect(2)
	elif cameraNumber == 3:
		cameraRaspi3.takePicture()
		time.sleep(2.5)
		camConnect(3)
	elif cameraNumber == 4:
		cameraRaspi4.takePicture()
		time.sleep(2.5)
		camConnect(4)
	buttonList[8].selected = False
	screen.blit(background, (130, 0))
	drawButtons()
	pygame.display.update()

def stopRover(fakeArg):	# button-based
	try:
		if indicatorList[4].active:
			driveControl.stopMotors()
	except:
		pass
	try:
		if indicatorList[5].active:
			armControl.stopMotors()
	except:
		pass

def runExperiment(fakeArg):	# button-based
	return

def checkClient(client): # test connection
	return client.test()
	
def camConnect(cameraNumber): # button-based
	for cameraButton in buttonList[0:4]:
		if cameraButton.selected:
			return
	if not indicatorList[cameraNumber - 1].active:
		return
	buttonList[4].selected = False
	buttonList[cameraNumber - 1].selected = True
	drawButtons()
	screen.blit(cameraSplash, (130, 0))
	pygame.display.update()
	if(os.name == "posix"): # linux machine
		command = ("nc -l -p 3001 | mplayer -really-quiet -xy 0.5 -nosound -hardframedrop -noautosub -fps 40 -ontop -noborder -geometry 156:58 -demuxer h264es -nocache -")
		subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
	else: #windows
		command = "cam.bat" # EXTERNAL FILE. Needs to be kept up to date.
		subprocess.Popen(str(command), shell=True, stdin=None, stdout=None, stderr=None)
	time.sleep(0.5)
	if(cameraNumber == 1):
		buttonList[0].selected = True
		cameraRaspi1.startCamera()
	elif(cameraNumber == 2):
		buttonList[1].selected = True
		cameraRaspi2.startCamera()
	elif(cameraNumber == 3):
		buttonList[2].selected = True
		cameraRaspi3.startCamera()
	elif(cameraNumber == 4):
		buttonList[3].selected = True
		cameraRaspi4.startCamera()
	redrawTimer = pygame.time.get_ticks()
	drawButtons()

def camDisconnect(fakeArg): # button-based
	if(buttonList[0].selected):
		cameraRaspi1.stopCamera()
	elif(buttonList[1].selected):
		cameraRaspi2.stopCamera()
	elif(buttonList[2].selected):
		cameraRaspi3.stopCamera()
	elif(buttonList[3].selected):
		cameraRaspi4.stopCamera()
	for cameraButton in buttonList[0:4]:
		cameraButton.selected = False
	buttonList[4].selected = True
	if(os.name == "posix"): # linux machine
		command = "sudo killall nc; sudo killall mplayer"
		subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
	else: #windows
		command = "camStop.bat" # EXTERNAL FILE. Needs to be kept up to date.
		subprocess.Popen(str(command), shell=True, stdin=None, stdout=None, stderr=None)
	drawButtons()

def connectClients(fakeArg): # button-based
	buttonList[10].selected = True
	drawButtons()
	drawIndicators()
	pygame.display.update()
	if not indicatorList[0].active:
		cameraRaspi1.connect(0);
	if not indicatorList[1].active:
		cameraRaspi2.connect(0);
	if not indicatorList[2].active:
		cameraRaspi3.connect(0);
	if not indicatorList[3].active:
		cameraRaspi4.connect(0);
	if not indicatorList[4].active:
		driveControl.connect(0);
	if not indicatorList[5].active:
		armControl.connect(0);
	buttonList[10].selected = False
	drawIndicators()
	drawButtons()
	pygame.display.update()


def quit(fakeArg): # button-based
	buttonList[11].selected = True
	buttonList[11].draw(screen)
	pygame.display.update()
	camDisconnect(None)
	pygame.quit()


	# main execution #############################################################################

cameraRaspi1 = CameraClient(IPraspi1, cameraClientPort)
cameraRaspi2 = CameraClient(IPraspi2, cameraClientPort)
cameraRaspi3 = CameraClient(IPraspi3, cameraClientPort)
cameraRaspi4 = CameraClient(IPraspi4, cameraClientPort)
driveControl = DriveClient(IPraspi2, driveClientPort)
armControl = ArmClient(IPraspi2, armClientPort)

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (30,30)
pygame.init()
pygame.display.set_caption("USST Rover GUI")
logo = pygame.image.load('logo.png')
pygame.display.set_icon(logo)
Clock = pygame.time.Clock()
screen = pygame.display.set_mode((1220, 700), pygame.NOFRAME)
background = pygame.image.load("background.jpg")
cameraSplash = pygame.image.load("camera.jpg")
screen.blit(background, (130, 0))


try:
	Controller.init()
except:
	pass

createBoxes()
drawBoxes()
createButtons()
drawButtons()
createIndicators()
drawIndicators()
#createRobot()
#robot.draw(screen)

pygame.display.update()
mainloop = True

setMode("drive")

indicatorTimer = 0
redrawTimer = 0
controllerSendTimer = 0

connectConsole()

while mainloop:
	mouse = pygame.mouse.get_pos()
	
	if pygame.time.get_ticks() - redrawTimer > 4000: # whole display redraw timer
		drawBoxes()
		screen.blit(background, (130, 0))
		drawButtons()
		drawIndicators()
		redrawTimer = pygame.time.get_ticks()
	
	if pygame.time.get_ticks() - controllerSendTimer > 250: # controller data sending timer
		if Controller.isConnected:
			getInput()
			if controlMode == "drive":
				if indicatorList[4].active:
					throttle = int(axes[1] * 128) + 128
					throttle = max(throttle, 0)
					throttle = min(throttle, 255)
					steering = int(axes[0] * 128) + 128
					steering = max(steering, 0)
					steering = min(steering, 255)
					driveControl.sendControlData(throttle, steering)
			elif controlMode == "arm":
				pass
		controllerSendTimer = pygame.time.get_ticks()
		
	Clock.tick(30)
	output.draw(screen)
	error.draw(screen)
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			quit(None)
		elif event.type == pygame.KEYDOWN:
			if event.key == pygame.K_ESCAPE:
				quit(None)
		elif event.type == pygame.MOUSEBUTTONDOWN:
			mouse = pygame.mouse.get_pos()
			for button in buttonList:
				if(button.obj.collidepoint(mouse)):
					button.press()
	
	pygame.display.update()

