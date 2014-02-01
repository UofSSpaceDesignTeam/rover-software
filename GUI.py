#!/usr/bin/python

# The main program for controlling the rover
# Added by Dylan

	# dependency list

import pygame
import Controller
from Button import Button
from Box import Box
from Indicator import Indicator
from DriveClient import DriveClient
from CameraClient import CameraClient
from ArmClient import ArmClient
from TextOutput import TextOutput
import socket
import commands
import os
import signal
import time
import sys
from datetime import date
import subprocess
from threading import Thread


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
colorGray = (125, 125, 125)
colorBlack = (0, 0, 0)
colorRed = (240, 0, 0)
colorGreen = (0, 240, 0)
colorDarkGreen = (0, 100, 0)
colorBlue = (0, 0, 240)
colorDarkBlue = (0, 0, 120)
colorYellow = (250, 250, 0)


	# function definitions

def createButtons():
	global buttonList
	buttonList = []
	
	camera1Button = Button(camConnect, (1), "Camera 1", 20, colorBlack, (12, 20, 100, 20), colorBlue, colorYellow)
	camera2Button = Button(camConnect, (2), "Camera 2", 20, colorBlack, (12, 50, 100, 20), colorBlue, colorYellow)
	camera3Button = Button(camConnect, (3), "Camera 3", 20, colorBlack, (12, 80, 100, 20), colorBlue, colorYellow)
	camera4Button = Button(camConnect, (4), "Camera 4", 20, colorBlack, (12, 110, 100, 20), colorBlue, colorYellow)
	cameraStopButton = Button(camDisconnect, (0), "None", 20, colorBlack, (12, 140, 100, 20), colorBlue, colorYellow)
	cameraStopButton.selected = True
	buttonList.append(camera1Button)	# 0
	buttonList.append(camera2Button)	# 1
	buttonList.append(camera3Button)	# 2
	buttonList.append(camera4Button)	# 3
	buttonList.append(cameraStopButton)	# 4
	
	moveButton = Button(setMode, ("drive"), "Drive", 20, colorBlack, (12, 195, 100, 20), colorBlue, colorYellow)
	armButton = Button(setMode, ("arm"), "Arm", 20, colorBlack, (12, 225, 100, 20), colorBlue, colorYellow)
	moveButton.selected = True
	buttonList.append(moveButton)	# 5
	buttonList.append(armButton)	# 6
	
	stopButton = Button(stopRover, None, "Stop", 24, colorYellow, (12, 285, 100, 20), colorRed, colorRed)
	pictureButton = Button(takePicture, None, "Picture", 20, colorWhite, (12, 315, 100, 20), colorBlue, colorYellow)
	runExperimentButton = Button(runExperiment, None, "Science", 20, colorWhite, (12, 345, 100, 20), colorBlue, colorYellow)
	buttonList.append(stopButton)	# 7
	buttonList.append(pictureButton)	# 8
	buttonList.append(runExperimentButton)	# 9
	
	connectButton = Button(connectClients, None, "Connect All", 24, colorWhite, (1120, 320, 120, 30), colorBlue, colorGreen)
	buttonList.append(connectButton) #10


def createBoxes():
	global boxList
	boxList = []
	
	cameraButtonBox = Box("Camera Feeds", 20, colorWhite, (0, 0, 125, 170), (17, 2), colorGray)
	boxList.append(cameraButtonBox)
	
	controlBox = Box("Control Modes", 20, colorWhite, (0, 175, 125, 80), (16, 177), colorGray)
	boxList.append(controlBox)
	
	actionBox = Box("Actions", 22, colorWhite, (0, 260, 125, 110), (35, 264), colorGray)
	boxList.append(actionBox)
	
	connectionsBox = Box("Connections", 24, colorWhite, (1095, 0, 160, 360), (1115, 8), colorDarkBlue)
	boxList.append(connectionsBox)


def createIndicators():
	global indicatorList
	indicatorList = []
	
	camera1Indicator = Indicator(checkClient, cameraRaspi1, "Camera 1", colorWhite, (1110, 40), colorRed, colorGreen)
	camera2Indicator = Indicator(checkClient, cameraRaspi2, "Camera 2", colorWhite, (1110, 80), colorRed, colorGreen)
	camera3Indicator = Indicator(checkClient, cameraRaspi3, "Camera 3", colorWhite, (1110, 120), colorRed, colorGreen)
	camera4Indicator = Indicator(checkClient, cameraRaspi4, "Camera 4", colorWhite, (1110, 160), colorRed, colorGreen)
	
	driveIndicator = Indicator(checkClient, driveControl, "Drive System", colorWhite, (1110, 200), colorRed, colorGreen)
	armIndicator = Indicator(checkClient, armControl, "Arm System", colorWhite, (1110, 240), colorRed, colorGreen)
	
	controllerIndicator = Indicator(checkController, None, "Controller", colorWhite, (1110, 280), colorRed, colorGreen)
	
	indicatorList.append(camera1Indicator) #0
	indicatorList.append(camera2Indicator) #1
	indicatorList.append(camera3Indicator) #2
	indicatorList.append(camera4Indicator) #3
	indicatorList.append(driveIndicator) #4
	indicatorList.append(armIndicator) #5
	indicatorList.append(controllerIndicator) #6

def connectConsole():
	global output
	global error
	output = TextOutput(15,colorGreen,30,540,350,160,10,colorWhite)
	error = TextOutput(15,colorRed,380,540,350,160,10,colorWhite)
	#t = threading.Thread(target=output.loop,args =())
	#t.start()
	#connect stderr and stdout to output
	sys.stdout = output
	sys.stderr = error
	
def drawButtons():
	for i in buttonList:
		i.draw(screen)


def drawBoxes():
	for i in boxList:
		i.draw(screen)
	screen.blit(background, (130, 0))


def drawIndicators():
	for i in indicatorList:
		i.refresh()
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


def checkController(fakeArg): # button-based
	if Controller.isConnected:
		return True
	else:
		return False


def getInput():
	global axes, buttons, dPad
	axes = Controller.getAxes()
	buttons = Controller.getButtons()
	dPad = Controller.getDPad()


def takePicture(fakeArg):	# button-based
	return


def stopRover(fakeArg):	# button-based
	driveControl.stopMotors()
	armControl.stopMotors()


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
	if(os.name == "posix"): # linux machine
		command = ("nc -l -p 3001 | mplayer -x 960 -y 540 -nosound -quiet -hardframedrop -noautosub -fps 40 -ontop -noborder -geometry 155:30 -demuxer h264es -nocache -")
		p = subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
	else: #windows
		command = "cam.bat" # EXTERNAL FILE. Needs to be kept up to date.
		p = subprocess.Popen(str(command), shell=True, stdin=None, stdout=None, stderr=None)
	time.sleep(0)
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
	drawBoxes()
	drawButtons()
	drawIndicators()


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
		command = ("sudo killall nc; sudo killall mplayer")
		subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
	else: #windows
		command = "camStop.bat" # EXTERNAL FILE. Needs to be kept up to date.
		subprocess.Popen(str(command), shell=True, stdin=None, stdout=None, stderr=None)
	drawButtons()


def connectClients(fakeArg): # button-based
	buttonList[10].selected = True
	drawButtons()
	pygame.display.update()
	for client in clientList:
		client.connect(0)
	buttonList[10].selected = False
	drawIndicators()
	pygame.display.update()



	# main execution #############################################################################

global clientList
clientList = []

driveControl = DriveClient(IPraspi2, driveClientPort)
armControl = ArmClient(IPraspi2, armClientPort)
cameraRaspi1 = CameraClient(IPraspi1, cameraClientPort)
cameraRaspi2 = CameraClient(IPraspi2, cameraClientPort)
cameraRaspi3 = CameraClient(IPraspi3, cameraClientPort)
cameraRaspi4 = CameraClient(IPraspi4, cameraClientPort)

clientList.append(driveControl)
clientList.append(armControl)
clientList.append(cameraRaspi1)
clientList.append(cameraRaspi2)
clientList.append(cameraRaspi3)
clientList.append(cameraRaspi4)

os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (25,30)
pygame.init()
logo = pygame.image.load('logo.png')
pygame.display.set_icon(logo)
Clock = pygame.time.Clock()
screen = pygame.display.set_mode((1260, 700))
background = pygame.image.load("background.png")

try:
	Controller.init()
except:
	pass

connectConsole()
createBoxes()
drawBoxes()
createButtons()
drawButtons()
createIndicators()
drawIndicators()

pygame.display.update()
mainloop = True

setMode("drive")

indicatorTimer = 0
redrawTimer = 0
controllerSendTimer = 0

while mainloop:
	mouse = pygame.mouse.get_pos()
	
	if pygame.time.get_ticks() - redrawTimer > 4000: # whole display redraw timer
		drawBoxes()
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
	pygame.display.set_caption("USST Rover GUI ("+ str(round(Clock.get_fps())) + " fps)")
	output.draw(screen)
	error.draw(screen)
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
