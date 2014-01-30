# Used on the Pi to controll drive motors

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

# Serial Variables
sBaudrate=9600
sTimeout=.2

# Server Variables
drivePort=3002
global clientAddress

def beginSerial():
	drive=serial.Serial("/dev/ttyAMA0")
	drive.baudrate = sBaudrate
	drive.timeout = sTimeout
	print(drive.name)
	drive.write("1,start\r\n")
	drive.write("2,start\r\n")
	print("Left/Right start commands sent!")
	return drive

# Currently Reserved for serial multiplexing
def beginGPIO():
#	GPIO.cleanup()
#	GPIO.setmode(GPIO.BOARD)
#	GPIO.setup(7,GPIO.OUT)
#	GPIO.setup(11,GPIO.OUT)
#	print("GPIO Initialized")
	return

def sendSerial(leftThrust,rightThrust):
	leftMotor="1"
	rightMotor="2"
	Start=",s"
	End="\r\n"
	lSignal = str(leftMotor + Start + str(int(leftThrust*scaleFactor)) + End)
	rSignal = str(rightMotor + Start + str(int(rightThrust*scaleFactor)) + End)
	print(rSignal)
	print(rSignal)
	print(lSignal)
	drive.write(lSignal)
	drive.write(rSignal)
	drive.write("1,gets\r\n")
	feedback1=drive.readline()
	drive.write("2,gets\r\n")
	feedback2=drive.readline()
	print("Feedback: " + feedback1 + feedback2)
	if(feedback1 == "1,E1\r\n"):
		beginSerial()

def driveCommand(xAxis,yAxis):
	xMid=128
	yMid=128
	drive=yAxis-yMid
	steer=xAxis-xMid

	# This is wrong... :(
	#if(steer < 0):
	#	leftThrust=drive+steer
	#	rightThrust=drive-steer
	#elif(steer > 0):
	#	leftThrust=drive+steer
	#	rightThrust=drive-steer

	if(steer == 0):
		leftThrust=drive
		rightThrust=drive
	else:
		leftThrust=drive+steer
		rightThrust=drive-steer

	print(leftThrust,rightThrust)
	sendSerial(leftThrust,rightThrust)


def stopDrive(): 
	#add something here for emergency stop
	return
	
def resetDrive():
	#add something here for emergency restart
	#currently implemented autorestart in sendSerial function
	return

def parseController(command):
	if(command[0] == "#"): # is valid
		if(command[1] == "D"):
			if(command[2] == "D" and len(command) > 4): # Drive, Data
				xAxis = int(ord(command[3]))
				yAxis = int(ord(command[4]))
				print(xAxis,yAxis)
				driveCommand(xAxis,yAxis)
			elif(command[2] == "S"): # Drive, Stop
				stopDrive()

def stopSockets():
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
				# Parse joystick information and convert it to simpleSerial motor commands
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
	
