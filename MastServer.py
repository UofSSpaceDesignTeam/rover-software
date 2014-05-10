from ServoDriver import *
from gyroCamera import *
import socket
import time

mastPort = 3004

def parseCommand(command): # parses and executes remote commands
	if command != None:
		if len(command) > 2:
			if command[0] == "#": # is valid
				if command[1] == "M":
					if command[2] == "S": # Camera look
						y_dPad = int(ord(command[3])) - 2	#vertical d-Pad button
						x_dPad = int(ord(command[4])) - 2	#horizontal d-Pad button
						gyroEnable = int(ord(command[5]))
						#print(gyroEnable)
						#print("y_dPad / x_dPad is: %d %d" % (y_dPad, x_dPad))
						#gyroCam.stableDriveMode(True, y_dPad, x_dPad)
					elif command[2] == "S": # Stop
						stopServos()
	else: # command == none
		stopServos()

def stopServos():
	try:
		servoDriver.setServo(3, 1300)
	except:
		raise
		
def stopSockets():
	try:
		mastSocket.close()
	except:
		raise
	try:
		serverSocket.close()
	except:
		raise

def quit():
	stopServos()
	stopSockets()
	exit(0)
		
### Main Program  ###

# set up servo driver
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
					print data
					parseCommand(data)
			except socket.timeout:
				#if stabilize:
					#correctPitch();
		print("Mast Server disconnected.")
	
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	quit()
except:
	quit()
