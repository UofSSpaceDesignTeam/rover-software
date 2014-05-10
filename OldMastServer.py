from ServoDriver import *
from gyroCamera import *
import socket
import time

mastPort = 3004

def parseCommand(command): # parses and executes remote commands
	global emergency
	if command != None:
		if len(command) > 2:
			if command[0] == "#": # is valid
				if command[1] == "M":
					if command[2] == "C": # Camera look
						#yButton = int(ord(command[3])) - 2
						#setPitch(yButton)
						#xButton = int(ord(command[4])) - 2
						#setYaw(xButton)
						y_dPad = int(ord(command[3])) - 2	#vertical d-Pad button
						x_dPad = int(ord(command[4])) - 2	#horizontal d-Pad button
						gyroEnable = int(ord(command[5]))
						print gyroEnable
						print("y_dPad / x_dPad are: %d / %d" % (y_dPad, x_dPad))
						#gyroCam.stableDriveMode(gyroEnable, y_dPad, x_dPad)
					elif command[2] == "S": # Stop
						stopServos()
						print("Servos stopped.")
	else: # command == none
		stopServos()

def stopServos():
	servoDriver.setServo(3, 1300)
	#servoDriver.setServo(1, 1500)
		
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

## Test Code
#while True:
#	pos = raw_input('Pulse Length Position?')
#	try:
#		servoDriver.setServo(3, int(pos))
#	except:
#		print("NaN!")		
		
## Begin server connection		
		
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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
