from ServoDriver import *

mastPort = 3004

def setPitch(yButton):
	Pitch.setRelative(y*100)
	
def setYaw(xButton):
	if xButton == -1:
		servoDriver.setServo(1, 1050)
	elif xButton == 1:
		servoDriver.setServo(1, 1950)
	else: # Zero or some invalid value got through...	
		servoDriver.setServo(1, 1500)

def parseCommand(command): # parses and executes remote commands
	global emergency
	if command != None:
		if len(command) > 2:
			if command[0] == "#": # is valid
				if command[1] == "M":
					if command[2] == "C" # Camera look
						yButton = int(ord(command[3]))
						setPitch(yButton)
						xButton = int(ord(command[4]))
						setYaw(xButton)
					elif command[2] == "S": # Stop
						stopServos()
						print("Servos stopped.")
	else: # command == none
		stopServos()

def stopServos():
	servoDriver.setServo(0, 1500)
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
	Pitch = Servo(servoDriver, 0, 800, 2200, 1500)
	Yaw = Servo(servoDriver, 1, 1050, 1950, 1500)
except:
	print("Servo setup failed!)

# Test Code
#while True:
#	pos = raw_input('Pulse Length Position?')
#	try:
#		servoDriver.setServo(0, int(pos))
#	except:
#		print("NaN!")		
		
## Begin server connection		
		
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.bind(("", mastPort))
	serverSocket.listen(0)
	print("Drive Server listening on port " + str(mastPort))
	while(True):
		(mastSocket, clientAddress) = serverSocket.accept()
		print("Drive Server connected.")
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

