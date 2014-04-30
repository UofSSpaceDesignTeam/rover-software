# A script continuously run by the science pi.

# dependency list

import math
import socket
import time
import subprocess
from ServoDriver import *
from ADS1x15 import ADS1x15 # adc
from Adafruit_I2C import Adafruit_I2C
import RPi.GPIO as GPIO # for stepper and other stuff

# global constants

sciencePort = 3006

# function definitions

def runExperiment():
	calibrateStepper()
	

def readVoltage(): # reads adc and returns voltage in volts
	voltage = adc.readADCSingleEnded(1)
	voltage = voltage * 3.3 / 4096.0
	return voltage

def parseCommand(command):
	if command == "#RE":
		runExperiment()

def stopSockets(): # Stops sockets on error condition
	try:
		scienceSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass


### Main Program ###

# set up logging
try:
	logfile = open("/home/pi/scienceLogs/" + time.strftime("%m%d%H%M%S", time.localtime()) + ".log", "w")
except:
	print("science logging failed!")

# set up GPIOs
try:
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	#GPIO.setup(7, GPIO.OUT)
except:
	print("GPIO setup failed!")
	raise
	#subprocess.call("sudo reboot", shell = True)
	
# begin server connection
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.bind(("", sciencePort))
	serverSocket.listen(0)
	print("Science Server listening on port " + str(sciencePort))
	# main execution loop
	while(True):
		(scienceSocket, clientAddress) = serverSocket.accept()
		print("Science Server connected.")
		while(True):
			data = scienceSocket.recv(256)
			if(data == ""): # socket closing
				break
			else:
				parseCommand(data)
		print("Science Server disconnected.")
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	stopSockets()
	GPIO.cleanup()
except:
	stopSockets()
	GPIO.cleanup()
	raise
