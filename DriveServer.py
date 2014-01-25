# Used on the Pi to controll drive motors

import socket
import time
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
	if(len(command) > 2 and command[0] == "#"): # is valid
		if(command[1] == "D"):
			if(command[2] == "D"): # Drive, Data
				axis=command.partition(D)[2]
				print(axis)
			if(command[2] == "S"): # Drive, Stop
				stopDrive()


				
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
	subprocess.Popen("sudo python CameraServer.py", shell = True) # restart on connection failure
except:
	print(e)
	stopSockets()