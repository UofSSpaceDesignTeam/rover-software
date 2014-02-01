# Used on the Pi to control drive motors
# Added by Austin Shirley


import socket
import time
import subprocess
import serial
# Reserved for serial multiplexing.
#import RPi.GPIO as GPIO


# Motor Contols Variables
scaleFactor=.55
#deadzone=.1
#deadzoneDelay=.1
global xAxis
global yAxis
global emergency


# Serial Variables
sBaudrate=9600
sTimeout=.2


# Server Variables
drivePort=3002
global clientAddress


# Function Definitions
def beginSerial():
	drive=serial.Serial("/dev/ttyAMA0")
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


def sendSerial(leftThrust,rightThrust): # Encodes commands to Kangaroo Format and sends via Serial
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
	sendFeedback(lSignal,rSignal)

def sendFeedback(lSignal,rSignal): # Gets speed position back from wheel and looks for error
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

	print(leftThrust,rightThrust)
	sendSerial(leftThrust,rightThrust)

def stopDrive(): # Handles Emergency Stop
	drive.write("1,s0\r\n")
	drive.write("1,s0\r\n")
	emergency = True
	time.sleep(2)
	return

def parseController(command): # Parses Socket Data back to Axis positions
	if(command[0] == "#"): # is valid
		if(command[1] == "D"):
			if(command[2] == "D" and len(command) > 4): # Drive, Data
				xAxis = int(ord(command[3]))
				yAxis = int(ord(command[4]))
				print(xAxis,yAxis)
				driveCommand(xAxis,yAxis)
			elif(command[2] == "S"): # Drive, Stop
				stopDrive()

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
				break
			else:
				emergency = False
				parseController(data)
		print("Connection from " + str(clientAddress[0]) + " closed")

# Exception Handling
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	stopSockets()
except socket.error as e:
	print(e)
	stopSockets()
	time.sleep(2)
	subprocess.Popen("sudo python DriveServer.py", shell = True) # restart on connection failure
except:
	stopSockets()
	raise