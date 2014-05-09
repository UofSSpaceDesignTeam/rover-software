import math
import socket
import time
import subprocess
from ServoDriver import *
from ADS1x15 import ADS1x15 # adc
from Adafruit_I2C import Adafruit_I2C
import RPi.GPIO as GPIO # for stepper and other stuff

sciencePort = 3006

def runExperiment():
	scienceData = []
	calibrateStepper()
	for i in range(1440)
		scienceData.append(readVoltage)
		rotateStepper(1)
		time.sleep(0.2)

def readVoltage(): # reads adc and returns voltage in volts
	voltage = adc.readADCSingleEnded(1)
	voltage = voltage * 3.3 / 4096.0
	return voltage

def rotateStepper(amount): # +ve is cw
	if amount > 0:
		GPIO.output(11, False)
	else:
		GPIO.output(11, True)
	for i in range(0, amount):
		GPIO.output(12, True)
		time.sleep(0.1)
		GPIO.output(12, False)
		time.sleep(0.1)
	
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
	GPIO.setup(11, GPIO.OUT) # stepper direction
	GPIO.setup(13, GPIO.OUT) # stepper step
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
