# Used on the Pi to control drive motors
# Don't need sudo anymore?
# Added by Austin Shirley



import socket
import time
import subprocess
import serial

# Motor Control Variables
scaleFactor = 0.55
emergency = False


# Serial Variables
sBaudrate = 9600
sTimeout = 0.2


# Server Variables
drivePort = 3002
global clientAddress

# Sabertooth Variables
speedR = 0
speedL = 0
speed = 0

# Direction Commands (Right Forwards, Left Forwards etc)
RF = 0
RR = 1
LF = 4
LR = 5

# Motor Controller Adresses (m1: front wheels, m2 middle wheels etc)
m1 = 128
m2 = 129
m3 = 130

# Function Definitions
def beginSerial():
	drive = serial.Serial("/dev/ttyAMA0", bytesize=8, parity='N', stopbits=1, )
	drive.baudrate = sBaudrate
	drive.timeout = sTimeout
	print(drive.name)
	drive.write("1,start\r\n")
	drive.write("2,start\r\n")
	print("Left/Right start commands sent!")
	return drive

# Section Currently Reserved for serial multiplexing (Not Implemented)
#def beginGPIO():
#	GPIO.cleanup()
#	GPIO.setmode(GPIO.BOARD)
#	GPIO.setup(7,GPIO.OUT)
#	GPIO.setup(11,GPIO.OUT)
#	print("GPIO Initialized")
#	return




def sendKangaroo(leftThrust,rightThrust): # Encodes commands to Kangaroo Format and sends via Serial
	leftMotor="1"
	rightMotor="2"
	Start=",s"
	End="\r\n"
	lSignal = str(leftMotor + Start + str(int(leftThrust*scaleFactor)) + End)
	rSignal = str(rightMotor + Start + str(int(rightThrust*scaleFactor)) + End)
	print(rSignal)
	print(lSignal)
	drive.write(lSignal)
	drive.write(rSignal)
	kangarooFeedback(lSignal,rSignal)
	
def sendSabertooth(address, command, speed):
	checksum=int(address)+int(command)+int(speed) & 127
	drive.write(chr(int(address)))
	drive.write(chr(int(command)))
	drive.write(chr(int(speed)))
	drive.write(chr(int(checksum)))
	
def sabertoothKill():
	# Disclaimer, no extinct Tigers were harmed in the making of this function
	address=127
	while(address <= 135):
		sendSabertooth(address,RF,0)
		sendSabertooth(address,LF,0)
		address = address + 1
	print("All Motors Stopped (Hopefully)")

def kangarooFeedback(lSignal,rSignal): # Gets speed position back from wheel and looks for error
	global emergency
	drive.write("1,gets\r\n")
	feedbackL=drive.readline()
	drive.write("2,gets\r\n")
	feedbackR=drive.readline()
	print("Feedback: " + feedbackL + feedbackR)
	
	# Detecting if motor stops when it shouldn't etc
	# Encoder Errors > Emergency Failure > Wheel Jams
	# Currently only implemented for two wheels
	
	# Detecting wheel jam
	lwheelJam = 0
	rwheelJam = 0
	if(feedbackL == "1,s0\r\n"):
		if(lSignal != "1,s0\r\n"):
			lwheelJam = 1
	if(feedbackR == "2,s0\r\n"):
		if(rSignal != "2,s0\r\n"):
			rwheelJam = 1
	
	# Detecting a failed emergency stop
	if(emergency == True):
		if(feedbackL != "1,s0\r\n"):
			lwheelJam = 3
			print("EMERGENCY STOP FAILED!")
		if(feedbackR != "1,s0\r\n"):
			rwheelJam = 3
			print("EMERGENCY STOP FAILED!")
	
	# Encoder Errors
	if(feedbackL == "1,E1\r\n"):
		drive.write("1,start\r\n")
		lwheelJam = 4
		print("Attempting Restart")
	if(feedbackR == "2,E1\r\n"):
		drive.write("2,start\r\n")
		rwheelJam = 4
		print("Attempting Restart")
	
	# Send data back on the Socket
	driveSocket.send("#DE" + str(lwheelJam) + str(rwheelJam))

def driveCommand(xAxis,yAxis): # Converts controller position to Thrust commands
	xMid=128
	yMid=128
	drive=yAxis-yMid
	steer=xAxis-xMid
	if(steer == 0):
		leftThrust=drive
		rightThrust=drive
	else:
		leftThrust=drive+steer
		rightThrust=drive-steer
	if(leftThrust >= 0):
		sendSabertooth(m1,LF,abs(leftThrust))
	else:
		sendSabertooth(m1,LR,abs(leftThrust))
	if(rightThrust >= 0):
		sendSabertooth(m1,RF,abs(rightThrust))
	else:
		sendSabertooth(m1,RR,abs(rightThrust))
		
	#sendKangaroo(leftThrust,rightThrust)
	print(leftThrust,rightThrust)
	
	

def kangarooKill(): 	# Handles Emergency Stop
	global emergency
	drive.write("1,s0\r\n")
	drive.write("1,s0\r\n")
	emergency = True
	return

def parseController(command): # Parses Socket Data back to Axis positions
	global emergency
	if len(command) > 2:
		if(command[0] == "#"): # is valid
			if(command[1] == "D"):
				if(command[2] == "D" and len(command) > 4): # Drive, Data
					xAxis = int(ord(command[3]))
					yAxis = int(ord(command[4]))
					print(xAxis,yAxis)
					driveCommand(xAxis,yAxis)
					if(emergency == True):
						if(xAxis == 128 and yAxis == 128):
							emergency = False
							print("Resuming Control")
						else:
							print("Please centre the Controller Axis to continue")
							kangarooKill()
							sabertoothKill()
							time.sleep(2)
				elif(command[2] == "S"): # Drive, Stop
					print("emergency stop command received")
					kangarooKill()
					sabertoothKill()
					time.sleep(2)

def stopSockets(): # Stops sockets on error condition
	try:
		commandSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass

# Main Program

# Setup
drive=beginSerial()
#beginGPIO()

# Begin connection
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
	serverSocket.bind(("", drivePort))
	serverSocket.listen(0)
	print("DriveServer listening on port " + str(drivePort))
	while(True):
		(driveSocket, clientAddress) = serverSocket.accept()
		print("Connected to " + str(clientAddress[0])) 
		while(True):
			data = driveSocket.recv(256)
			if(data == ""): # socket closing
				kangarooKill()
				sabertoothKill()
				break
			else:
				parseController(data)
		print("Connection to " + str(clientAddress[0]) + " was closed")

# Exception Handling
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	kangarooKill()
	sabertoothKill()
	stopSockets()
except socket.error as e:
	print(e.strerror)
	kangarooKill()
	sabertoothKill()
	stopSockets()
	time.sleep(2)
	subprocess.Popen("sudo python DriveServer.py", shell = True) # restart on connection failure
except:
	kangarooKill()
	sabertoothKill()
	stopSockets()
	raise