from ServoDriver import *
import socket
import time

drivePort = 3002

def setSpeed(leftSpeed, rightSpeed):#4-6 left side 9-11 right side
	leftSpeed = int((leftSpeed*4.6)) + 1570
	rightSpeed = int((rightSpeed*4.6)) + 1570
	if leftSpeed > 2200:
		leftSpeed = 2200
	if leftSpeed < 900:
		leftSpeed = 900
	if rightSpeed > 2200:
		rightSpeed = 2200
	if rightSpeed < 900:
		rightSpeed = 900
	print leftSpeed
	print rightSpeed
	servoDriver.setServo(4,leftSpeed)
	servoDriver.setServo(5,leftSpeed)
	servoDriver.setServo(6,leftSpeed)
	servoDriver.setServo(9, rightSpeed)
	servoDriver.setServo(10, rightSpeed)
	servoDriver.setServo(11, rightSpeed)

def parseCommand(command): # parses and executes remote commands
	if command != None:
		if len(command) > 2:
			if command[0] == "#": # is valid
				if command[1] == "D":
					if command[2] == "1" and len(command) > 5: # one stick drive
						xChar = int(ord(command[3]))
						yChar = int(ord(command[4]))
						limit = int(ord(command[5]))
						leftSpeed = yChar + xChar - 254
						rightSpeed = yChar - xChar
						if max(abs(leftSpeed), abs(rightSpeed)) > limit:
							scaleFactor = float(limit) / max(abs(leftSpeed), abs(rightSpeed))
						else:
							scaleFactor = 1
						leftSpeed *= scaleFactor
						rightSpeed *= scaleFactor
						setSpeed(leftSpeed, rightSpeed)
					elif command[2] == "2" and len(command) > 4: # two stick drive
						leftSpeed = int(ord(command[3])) - 127
						rightSpeed = int(ord(command[4])) - 127
						setSpeed(leftSpeed, rightSpeed)
					elif command[2] == "S": # Stop
						stopServos()
						print("motors stopped.")
	else: # command == none
		stopServos()
		
def stopServos():
	servoDriver.setServo(4, 1570)
	servoDriver.setServo(5, 1570)
	servoDriver.setServo(6, 1570)
	servoDriver.setServo(9, 1570)
	servoDriver.setServo(10,1570)
	servoDriver.setServo(11,1570)
	
def stopSockets():
	try:
		driveSocket.close()
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
	
##Start Servos

try:
	servoDriver = ServoDriver()
except:
	print("Servo setup failed!")
	
# begin server connection
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	serverSocket.bind(("", drivePort))
	serverSocket.listen(0)
	print("Drive Server listening on port " + str(drivePort))
	while(True):
		(driveSocket, clientAddress) = serverSocket.accept()
		print("Drive Server connected.")
		driveSocket.settimeout(1.0)
		while(True):
			try:
				data = driveSocket.recv(256)
				if(data == ""): # socket closing
					stopServos()
					break
				else:
					parseCommand(data)
			except socket.timeout:
				stopServos()
		print("Drive Server disconnected.")
	
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	quit()
except:
	raise
	quit()
