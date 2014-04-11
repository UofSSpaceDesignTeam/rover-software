# The main program for controlling the rover from a base station. Requires Python 2.7 and pygame.

# dependency list

import sys
sys.dont_write_bytecode = True # stops .pyc generation in subsequent imports
	
import pygame
from Controller import Controller
from Button import Button
from Box import Box
from Indicator import Indicator
from DriveClient import DriveClient
from CameraClient import CameraClient
from ArmClient import ArmClient
from TextOutput import TextOutput
from RobotPiece import RobotPiece
import socket
import os
import time
from datetime import date
import subprocess

# global constants

IPraspi1 = "192.168.1.103"
IPraspi2 = "10.64.226.138" #"192.168.1.104"
IPraspi3 = "192.168.1.105"
IPraspi4 = "192.168.1.106"
# netcat-ed video on port 3001
driveClientPort = 3002
armClientPort = 3003
cameraClientPort = 3000
colorWhite = (255, 255, 255)
colorGreen = (0, 240, 0)
colorBlue = (0, 0, 240)
colorLightBlue = (100, 100, 250)
colorYellow = (250, 250, 0)

speedScale = 0.5
steeringTrim = 0.0

lastFix = 0
baseLocation = (0, 0)
roverLocation = (0, 0)

# function definitions

def createButtons(): # creates interactive buttons, and places them in a list in a defined order.
	global buttonList
	buttonList = []
	camera1Button = Button(camConnect, (1), "Front Camera", (12, 25, 100, 20), colorLightBlue, colorGreen)
	camera2Button = Button(camConnect, (2), "Arm Camera", (12, 55, 100, 20), colorLightBlue, colorGreen)
	camera3Button = Button(camConnect, (3), "Mast Camera", (12, 85, 100, 20), colorLightBlue, colorGreen)
	camera4Button = Button(camConnect, (4), "Rear Camera", (12, 115, 100, 20), colorLightBlue, colorGreen)
	cameraStopButton = Button(camDisconnect, (0), "Off", (12, 145, 100, 20), colorLightBlue, colorGreen)
	cameraStopButton.selected = True
	moveButton1 = Button(setDriveMode1, None, "1 Stick Drive", (12, 204, 100, 20), colorLightBlue, colorGreen)
	moveButton1.selected = True
	armButton1 = Button(setArmMode1, None, "Arm Base", (12, 264, 100, 20), colorLightBlue, colorGreen)
	stopButton = Button(stopRover, None, "Stop", (12, 354, 100, 20), colorLightBlue, colorYellow)
	pictureButton = Button(takePicture, None, "Take Picture", (12, 384, 100, 20), colorLightBlue, colorYellow)
	runExperimentButton = Button(runExperiment, None, "Science!", (12, 414, 100, 20), colorLightBlue, colorYellow)
	connectButton = Button(connectClients, None, "Connect All", (1107, 180, 100, 20), colorLightBlue, colorYellow)
	quitButton = Button(quit, None, "Quit", (12, 475, 100, 20), colorLightBlue, colorYellow)
	moveButton2 = Button(setDriveMode2, None, "2 Stick Drive", (12, 234, 100, 20), colorLightBlue, colorGreen)
	armButton2 = Button(setArmMode2, None, "Arm End", (12, 294, 100, 20), colorLightBlue, colorGreen)
	waypointButton = Button(setWaypoint, None, "Set Waypoint", (1107, 452, 100, 20), colorLightBlue, colorYellow)
	buttonList.append(camera1Button)	# 0
	buttonList.append(camera2Button)	# 1
	buttonList.append(camera3Button)	# 2
	buttonList.append(camera4Button)	# 3
	buttonList.append(cameraStopButton)	# 4
	buttonList.append(moveButton1)	# 5
	buttonList.append(armButton1)	# 6
	buttonList.append(stopButton)	# 7
	buttonList.append(pictureButton)	# 8
	buttonList.append(runExperimentButton)	# 9
	buttonList.append(connectButton)	# 10
	buttonList.append(quitButton)	# 11
	buttonList.append(moveButton2)	# 12
	buttonList.append(armButton2)	# 13
	buttonList.append(waypointButton) # 14

def createBoxes(): # creates simple graphical elements, and places them in a list
	global boxList
	boxList = []
	cameraButtonBox = Box("Camera Feeds", (0, 0, 125, 175))
	controlBox = Box("Control Modes", (0, 180, 125, 143))
	actionBox = Box("Rover Actions", (0, 328, 125, 115))
	uiBox = Box("User Interface", (0, 449, 125, 53))
	connectionsBox = Box("Connections", (1095, 0, 125, 209))
	gpsBox = Box("GPS", (1095, 339, 125, 143))
	controllerBox = Box("Controller", (1095, 214, 125, 120))
	boxList.append(cameraButtonBox)
	boxList.append(controlBox)
	boxList.append(actionBox)
	boxList.append(uiBox)
	boxList.append(connectionsBox)
	boxList.append(gpsBox)
	boxList.append(controllerBox)

def createIndicators(): # creates visual status indicators, and places them in a list in a defined order.
	global indicatorList
	indicatorList = []
	camera1Indicator = Indicator(testClient, cameraRaspi1, "Front Camera", (1106, 30))
	camera2Indicator = Indicator(testClient, cameraRaspi2, "Arm Camera", (1106, 55))
	camera3Indicator = Indicator(testClient, cameraRaspi3, "Mast Camera", (1106, 80))
	camera4Indicator = Indicator(testClient, cameraRaspi4, "Rear Camera", (1106, 105))
	driveIndicator = Indicator(testClient, driveControl, "Drive System", (1106, 130))
	armIndicator = Indicator(testClient, armControl, "Arm System", (1106, 155))
	controllerIndicator = Indicator(checkController, None, "Detected", (1114, 238))
	gpsIndicator = Indicator(checkGPS, None, "Active", (1118, 360))
	indicatorList.append(camera1Indicator) #0
	indicatorList.append(camera2Indicator) #1
	indicatorList.append(camera3Indicator) #2
	indicatorList.append(camera4Indicator) #3
	indicatorList.append(driveIndicator) #4
	indicatorList.append(armIndicator) #5
	indicatorList.append(controllerIndicator) #6
	indicatorList.append(gpsIndicator) #7

def createConsoles(): # set up the info boxes
	global output
	output = TextOutput("Messages", 17, colorWhite, (740, 544, 350, 156), 11)
	sys.stdout = output
	print("Welcome to the USST rover control system.")
	global gpsDisplay
	gpsDisplay = TextOutput("", 17, colorWhite, (1095, 361, 125, 85), 5)
	gpsDisplay.write("Mode: Absolute")
	gpsDisplay.write("Lat: Unknown")
	gpsDisplay.write("Lon: Unknown")
	gpsDisplay.write("Wpt Lat:")
	gpsDisplay.write("Wpt Lon:")
	global controllerDisplay
	controllerDisplay = TextOutput("", 17, colorWhite, (1112, 240, 88, 88), 5)

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
	
def createRobot():
	global robotPieceList
	robotPieceList = []
	chassis = RobotPiece(pygame.image.load('./graphics/chassis.png'), None, (1100,540))
	arm = RobotPiece(pygame.image.load('./graphics/arm.png'),pygame.image.load('./graphics/redArm.png'),(1145,515))
	wheels = RobotPiece(pygame.image.load('./graphics/twheelarray.png'),None,(1100,540))
	wheels.active = True
	arm.active = False
	chassis.active = True
	robotPieceList.append(chassis)
	robotPieceList.append(wheels)
	robotPieceList.append(arm)

def drawRobot():
	for piece in robotPieceList:
		piece.draw(screen)

def setDriveMode1(fakeArg):	# button-based
	# todo: stop arm movements
	buttonList[6].selected = False
	buttonList[12].selected = False
	buttonList[13].selected = False
	buttonList[5].selected = True
	drawButtons()
	
def setDriveMode2(fakeArg):	# button-based
	# todo: stop arm movements
	buttonList[5].selected = False
	buttonList[6].selected = False
	buttonList[13].selected = False
	buttonList[12].selected = True
	drawButtons()
	
def setArmMode1(fakeArg):	# button-based
	# todo: stop driving
	buttonList[5].selected = False
	buttonList[12].selected = False
	buttonList[13].selected = False
	buttonList[6].selected = True
	drawButtons()
	
def setArmMode2(fakeArg):	# button-based
	# todo: stop driving
	buttonList[5].selected = False
	buttonList[6].selected = False
	buttonList[12].selected = False
	buttonList[13].selected = True
	drawButtons()

def checkController(fakeArg):
	return controller.isConnected

def checkGPS(fakeArg):
	if indicatorList[4].active and pygame.time.get_ticks() - lastFix < 3000:
		return True
	else:
		return False

def setWaypoint(fakeArg):
	if indicatorList[7].active:
		buttonList[14].selected = True
		buttonList[14].draw(screen)
		pygame.display.update()
		# replace with actual things
		time.sleep(0.2)
		#
		buttonList[14].selected = False
		buttonList[14].draw(screen)

def takePicture(fakeArg):
	cameraNumber = 0
	for i in range(0, 4):
		if buttonList[i].selected:
			cameraNumber = i + 1
	if cameraNumber == 0:
		return
	buttonList[8].selected = True
	drawButtons()
	screen.blit(cameraSplash, (130, 0))
	pygame.display.update()
	camDisconnect(None)
	time.sleep(0.75)
	if cameraNumber == 1:
		cameraRaspi1.takePicture()
		time.sleep(2.5)
		cameraRaspi1.getPicture()
		time.sleep(2.5)
		camConnect(1)
	elif cameraNumber == 2:
		cameraRaspi2.takePicture()
		time.sleep(2.5)
		cameraRaspi2.getPicture()
		time.sleep(2.5)
		camConnect(2)
	elif cameraNumber == 3:
		cameraRaspi3.takePicture()
		time.sleep(2.5)
		cameraRaspi3.getPicture()
		time.sleep(2.5)
		camConnect(3)
	elif cameraNumber == 4:
		cameraRaspi4.takePicture()
		time.sleep(2.5)
		cameraRaspi4.getPicture()
		time.sleep(2.5)
		camConnect(4)
	time.sleep(0.75)
	buttonList[8].selected = False
	global redrawTimer
	redrawTimer = pygame.time.get_ticks()
	drawButtons()
	pygame.display.update()

def stopRover(fakeArg):	# button-based
	try:
		driveControl.stopMotors()
		armControl.stopMotors()
	except:
		pass

def runExperiment(fakeArg):	# button-based
	return

def testClient(client): # button-based
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
	if isLinux:
		command = ("nc -l -p 3001 | mplayer -really-quiet -xy 0.5 -nosound -hardframedrop -noautosub -fps 40 -ontop -noborder -geometry 150:48 -demuxer h264es -nocache -")
		subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
	else: # windows
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
	if isLinux: # linux machine
		command = "sudo killall nc; sudo killall mplayer"
		subprocess.Popen(str(command), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=None)
	else: #windows
		command = "camStop.bat" # EXTERNAL FILE. Needs to be kept up to date.
		subprocess.Popen(str(command), shell=True, stdin=None, stdout=None, stderr=None)
	drawButtons()

def connectClients(fakeArg): # button-based
	buttonList[10].selected = True
	buttonList[10].draw(screen)
	pygame.display.update()
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
	stopRover(None)
	pygame.quit()
	sys.exit(0)

# program execution starts here

if(os.name == "posix"): # linux machine
	isLinux = True
else:
	isLinux = False

# create communication clients
cameraRaspi1 = CameraClient(IPraspi1, cameraClientPort)
cameraRaspi2 = CameraClient(IPraspi2, cameraClientPort)
cameraRaspi3 = CameraClient(IPraspi3, cameraClientPort)
cameraRaspi4 = CameraClient(IPraspi4, cameraClientPort)
driveControl = DriveClient(IPraspi1, driveClientPort)
armControl = ArmClient(IPraspi2, armClientPort)

# set up pygame resources
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (20, 20)
pygame.init()
pygame.display.set_caption("USST Rover GUI")
logo = pygame.image.load('./graphics/logo.png')
pygame.display.set_icon(logo)
Clock = pygame.time.Clock()
screen = pygame.display.set_mode((1220, 700), pygame.NOFRAME)
background = pygame.image.load("./graphics/background.jpg")
cameraSplash = pygame.image.load("./graphics/camera.jpg")
screen.blit(background, (130, 0))

# initialize everything
controller = Controller(0)
createBoxes()
createButtons()
createIndicators()
createRobot()
createConsoles()
drawBoxes()
drawButtons()
gpsDisplay.draw(screen)
drawIndicators()
drawRobot()
output.draw(screen)
pygame.display.update()
redrawTimer = 0
controllerSendTimer = 0

if controller.isConnected:
	print("Controller is connected.")
else:
	print("Controller is not connected.")

while True: # main execution loop

	# check scheduled tasks
	if pygame.time.get_ticks() - redrawTimer > 5000: # whole display redraw timer
		redrawTimer = pygame.time.get_ticks()
		screen.blit(background, (130, 0))
		drawBoxes()
		drawButtons()
		gpsDisplay.draw(screen)
		drawIndicators()
	if pygame.time.get_ticks() - controllerSendTimer > 200: # control data send timer
		controllerSendTimer = pygame.time.get_ticks()
		output.draw(screen) # also refresh the message display
		if controller.isConnected:
			axes = controller.getAxes()
			#buttons = controller.getButtons()
			#dPad = controller.getDPad()
			if isLinux:
				controllerDisplay.write("Left X: " + str(round(axes[0], 2)))
				controllerDisplay.write("Left Y: " + str(round(axes[1], 2)))
				controllerDisplay.write("Right X: " + str(round(axes[2], 2)))
				controllerDisplay.write("Right Y: " + str(round(axes[3], 2)))
				controllerDisplay.write("Trigger: " + str(round(axes[4], 2)))
			else:
				controllerDisplay.write("Left X: " + str(round(axes[0], 2)))
				controllerDisplay.write("Left Y: " + str(round(axes[1], 2)))
				controllerDisplay.write("Right X: " + str(round(axes[2], 2)))
				controllerDisplay.write("Right Y: " + str(round(axes[3], 2)))
				controllerDisplay.write("Trigger: " + str(round(axes[4], 2)))
			controllerDisplay.draw(screen)
			indicatorList[6].draw(screen)
			if buttonList[5].selected: # 1 stick drive mode
				if indicatorList[4].active: # connected
					if isLinux:
						driveControl.sendOneStickData(axes[0] * speedScale, axes[1] * speedScale)
					else: # wandows
						driveControl.sendOneStickData(axes[0] * speedScale, axes[1] * speedScale)
			elif buttonList[12].selected: # 2 stick drive mode
				if indicatorList[4].active: # connected
					if isLinux:
						driveControl.sendTwoStickData(axes[1] * speedScale, axes[3] * speedScale)
					else:
						driveControl.sendTwoStickData(axes[1] * speedScale, axes[3] * speedScale)
			elif buttonList[6].selected: # arm mode 1
				if indicatorList[5].active:
					wristPan = int(axes[2] * 80) + 127
					if wristPan != 127:
						armControl.panHand(wristPan)
						time.sleep(0.005)
					wristTwist = 127 - int(axes[4] * 120)
					if wristTwist != 127:
						armControl.twistHand(wristTwist)
						time.sleep(0.005)
					wristTilt = 127 - int(axes[3] * 17)
					if wristTilt != 127:
						armControl.tiltWrist(wristTilt)
						time.sleep(0.005)
					wristLift = int(axes[0]*127)+127
					armControl.liftWrist(wristLift)
					time.sleep(0.005)
					wristMove = int(axes[1]*127) +127
					#if wristMove != 127:
					armControl.moveWrist(wristMove)
					time.sleep(0.005)
					basePan = int(axes[2]*127) + 127
					armControl.panBase(basePan)
					time.sleep(0.005)
			if buttonList[13].selected: # temporary test actuator mode (2)
				if indicatorList[5].active: # arm mode
					throttle = int(axes[1] * 127) + 127
					throttle = max(throttle, 0)
					throttle = min(throttle, 254)
					steering = int(axes[3] * 127) + 127
					steering = max(steering, 0)
					steering = min(steering, 254)
					armControl.temp_actuator1(throttle, steering)
	
	# update UI state, check events
	mouse = pygame.mouse.get_pos()
	Clock.tick(30)
	for event in pygame.event.get():
		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_ESCAPE:
				quit(None)
		elif event.type == pygame.QUIT:
			quit(None)
		if event.type == pygame.MOUSEBUTTONDOWN:
			mouse = pygame.mouse.get_pos()
			#print(str(mouse))
			for button in buttonList:
				if(button.obj.collidepoint(mouse)):
					button.press()
	pygame.display.update()

