from ServoDriver import *
import socket
import time

drivePort = 3002

def setSpeed(leftSpeed, rightSpeed):
	if leftSpeed > 2000:
		leftSpeed = 2000
	if leftSpeed < 1000:
		leftSpeed = 1000
	if rightSpeed > 2000:
		rightSpeed = 2000
	if rightSpeed < 1000:
		rightSpeed = 1000
	servoDriver,setServo(4,leftSpeed)
	servoDriver,setServo(5, rightSpeed)

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
						setMotors(leftSpeed, rightSpeed)
					elif command[2] == "2" and len(command) > 4: # two stick drive
						leftSpeed = int(ord(command[3])) - 127
						rightSpeed = int(ord(command[4])) - 127
						setMotors(leftSpeed, rightSpeed)
						watchdog = pygame.time.get_ticks()
					elif command[2] == "S": # Stop
							stopServos()
						print("motors stopped.")
	else: # command == none
		stopServos()
		
def stopServos():
	servoDriver.setServo(4, 1570)
	servoDriver.setServo(5, 1570)
	
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
	quit()