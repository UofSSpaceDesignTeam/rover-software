# Used on the Pi to controll drive motors

import socket
import time
import subprocess
#import serial
# Reserved for serial multiplexing.
#import RPi.GPIO as GPIO

# Motor Contols Variables
#scaleFactor=1000
#deadzone=.1
#deadzoneDelay=.1
global xAxis
global yAxis

# Serial Variables
sBaudrate=9600
sTimeout=3

# Server Variables
drivePort=3002
global clientAddress

def beginSerial():
	drive=serial.Serial("/dev/ttyAMA0")
	motor.baudrate = sBaudrate
	motor.timeout = sTimeout
	print(motor.name)
	motor.write("1,start\r\n")
	print("Start command sent!")
	return motor

# Currently Reserved for serial multiplexing
def beginGPIO():
#	GPIO.cleanup()
#	GPIO.setmode(GPIO.BOARD)
#	GPIO.setup(7,GPIO.OUT)
#	GPIO.setup(11,GPIO.OUT)
#	print("GPIO Initialized")
	return

def driveCommand(axis):
	leftCenter=0

def stopDrive():
	return

def parseController(command):
	if(command[0] == "#"): # is valid
		if(command[1] == "D"):
			if(command[2] == "D" and len(command) > 4): # Drive, Data
				xAxis = int(ord(command[3]))
				yAxis = int(ord(command[4]))
				print(xAxis,yAxis)
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
#beginSerial()
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
	
