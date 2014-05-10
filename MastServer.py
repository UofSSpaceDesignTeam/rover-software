from ServoDriver import *
from IMU import *
import socket
import time

mastPort = 3004

def setPitch(yButton):
	global currentPitch
	if yButton == -1:
		if currentPitch > 800:
			for x in range (0, 50):
				currentPitch = currentPitch - 1
				servoDriver.setServo(3, currentPitch)
	elif yButton == 1:
		if currentPitch < 2300:
			for x in range (0, 50):
				currentPitch = currentPitch + 1
				servoDriver.setServo(3, currentPitch)
	
def setYaw(xButton):
	if xButton == -1:
		servoDriver.setServo(1, 1480)
	elif xButton == 1:
		servoDriver.setServo(1, 1520)
	else: # Zero or some invalid value got through...	
		servoDriver.setServo(1, 1500)

def parseCommand(command): # parses and executes remote commands
	if command != None:
		if len(command) > 2:
			if command[0] == "#": # is valid
				if command[1] == "M":
					if command[2] == "S": # Camera look
						panValue = int(ord(command[3]))
						if panValue > 127:
							pan.setAbsolute(1520)
						elif panValue < 127:
							pan.setAbsolute(1480)
						else:
							pan.setAbsolute(1500)
						tilt.setRelative(int(ord(command[4])))
					elif command[2] == "S": # Stop
						stopServos()
	else: # command == none
		stopServos()

def stopServos():
	try:
		pan.setAbsolute(1500)
		tilt.setAbsolute(1300)
	except:
		pass
		
def stopSockets():
	try:
		mastSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass

def quit():
	stopServos()
	stopSockets()
	exit(0)
		
### Main Program  ###

# set up servo driver
try:
	servoDriver = ServoDriver()
	pan = Servo(servoDriver, 1, 1050, 1950, 1500)
	tilt = Servo(servoDriver, 3, 800, 2300, 1300)
except:
	print("Servo setup failed!")
	quit()
		
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.bind(("", mastPort))
	serverSocket.listen(0)
	print("Mast Server listening on port " + str(mastPort))
	while(True):
		(mastSocket, clientAddress) = serverSocket.accept()
		print("Mast Server connected.")
		mastSocket.settimeout(0.5)
		while(True):
			try:
				data = mastSocket.recv(256)
				if(data == ""): # socket closing
					stopServos()
					break
				else:
					parseCommand(data)
			except socket.timeout:
				if stabilize:
					correctPitch();
		print("Mast Server disconnected.")
	
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	quit()
except:
	quit()
