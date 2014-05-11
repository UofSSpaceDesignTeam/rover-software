from ServoDriver import *
from gyroCamera import *
import socket
import time

mastPort = 3004
startPitch = 100
currentPitch = 1300

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
	global emergency
	if command != None:
		if len(command) > 2:
			if command[0] == "#": # is valid
				if command[1] == "M":
					if command[2] == "C": # Camera look
						yButton = int(ord(command[3])) - 2
						xButton = int(ord(command[4])) - 2
						gyroCam.stableDriveMode(True, yButton, xButton)
						#time.sleep(0.1)
					elif command[2] == "S": # Stop
						stopServos()
						print("Servos stopped.")
	else: # command == none
		stopServos()

def stopServos():
	servoDriver.setServo(3, 1300)
	servoDriver.setServo(1, 1500)
		
def stopSockets():
	try:
		mastSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass

		
## Start Servos
try:
	servoDriver = ServoDriver()
	# Start the gyrocamera with the servoDriver
	try:
		gyroCam = GyroCamera(servoDriver)
	except:
		print("Gyro-Camera setup failed!")
		raise
	#Pitch = Servo(servoDriver, 3, 800, 2300, 1400)
	#Yaw = Servo(servoDriver, 1, 1050, 1950, 1500)
except:
	print("Servo setup failed!")

## Begin server connection		
		
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	serverSocket.bind(("", mastPort))
	serverSocket.listen(0)
	print("Mast Server listening on port " + str(mastPort))
	while(True):
		(mastSocket, clientAddress) = serverSocket.accept()
		print("Mast Server connected.")
		while(True):
			data = mastSocket.recv(256)
			if(data == ""): # socket closing
				stopServos()
				break
			else:
				parseCommand(data)
		print("Mast Server disconnected.")
	
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	stopServos()
	stopSockets()
except:
	stopServos()
	stopSockets()
	raise
