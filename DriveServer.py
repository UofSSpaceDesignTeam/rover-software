# A script continuously run by the drive control Pi.

# dependency list

import socket
import time
import subprocess
import serial
import RPi.GPIO as GPIO # for hardware reset system

# global constants	

drivePort = 3002
scaleFactor = 0.55
deadband = 0
ramping = 20
commandRF = 0
commandRR = 1
commandLF = 4
commandLR = 5
controllerAddress = [128, 129, 130]
steermult = .88
skidmode = 1

# global variables

emergency = False

# Function Definitions

def sendSabertooth(address, command, speed):
	checksum = int(address) + int(command) + int(speed) & 127
	controller.write(chr(int(address)))
	controller.write(chr(int(command)))
	controller.write(chr(int(speed)))
	controller.write(chr(int(checksum)))
	
def stopSabertooth():
	for address in controllerAddress:
		sendSabertooth(address,commandRF,0)
		sendSabertooth(address,commandLF,0)

def setMotors(xAxis, yAxis): # sends motor commands based on joystick position
	throttle = (yAxis - 127) / 127.0  # range is now -1 to 1
	steering = (xAxis - 127) / 127.0
	
	# Math for SkidSteer
	leftSpeed = (throttle - (steering * steermult)) * 127
	rightSpeed = (throttle + (steering * steermult)) * -127

	leftSpeed = max(leftSpeed, -127)
	leftSpeed = min(leftSpeed, 127)
	rightSpeed = max(rightSpeed, -127)
	rightSpeed = min(rightSpeed, 127)
	
	# send forward / reverse commands to controllers
	if(leftSpeed >= 0):
		for address in controllerAddress:
			sendSabertooth(address, commandLF, leftSpeed)
	else:
		for address in controllerAddress:
			sendSabertooth(address, commandLR, -1 * leftSpeed)
			
	if(rightSpeed >= 0):
		for address in controllerAddress:
			sendSabertooth(address, commandRF, rightSpeed)
	else:
		for address in controllerAddress:
			sendSabertooth(address, commandRR, -1 * rightSpeed)

def setMotors2(yAxisr, yAxisl): # sends motor commands based on joystick1/2 y position
	#NOTE: xAxis is now read as yAxisr.  Make sure to change this in GUI.
	throttlel = (yAxisl - 127) / 127.0  # range is now -1 to 1
	throttler = (yAxisr - 127) / 127.0

	# Math for SkidSteer
	leftSpeed = (throttlel) * 127
	rightSpeed = (throttler) * 127
	
	leftSpeed = max(leftSpeed, -127)
	leftSpeed = min(leftSpeed, 127)
	rightSpeed = max(rightSpeed, -127)
	rightSpeed = min(rightSpeed, 127)
	
	# send forward / reverse commands to controllers
	if(leftSpeed >= 0):
		for address in controllerAddress:
			sendSabertooth(address, commandLF, leftSpeed)
	else:
		for address in controllerAddress:
			sendSabertooth(address, commandLR, -1 * leftSpeed)
			
	if(rightSpeed >= 0):
		for address in controllerAddress:
			sendSabertooth(address, commandRF, rightSpeed)
	else:
		for address in controllerAddress:
			sendSabertooth(address, commandRR, -1 * rightSpeed)
			
def parseCommand(command): # parses and executes remote commands
	global emergency
	global skidmode
	if len(command) > 2:
		if(command[0] == "#"): # is valid
			if(command[1] == "D"):
				if(command[2] == "D" and len(command) > 4): # Drive, Data
					Axis1 = int(ord(command[3]))
					Axis2 = int(ord(command[4]))
					print(Axis1, Axis2) #x+y in mode 1 or ry+ly in mode 2
					if(skidmode == 1):
						setMotors(Axis1, Axis2)
					if(skidmode == 2):
						setMotors2(Axis1, Axis2)
					if(emergency == True):
						if(Axis1 == 127 and Axis2 == 127):
							emergency = False
						else:
							print("Check controller connection and release joystick.")
							stopSabertooth()
							time.sleep(2)
				if(command[2] == "M"): # Mode Switch
					skidmode = int(ord(command[3]))
					print str(skidmode) + "selected"
				elif(command[2] == "S"): # Stop
					stopSabertooth()
					print("emergency stop")
					emergency = True
					

def stopSockets(): # Stops sockets on error condition
	try:
		driveSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass


### Main Program  ###

# set up motor controllers
try:
	controller = serial.Serial("/dev/ttyAMA0", bytesize = 8, parity = 'N', stopbits = 1)
	controller.baudrate = 9600
	controller.timeout = 0.2
	for address in controllerAddress:
		sendSabertooth(address, 16, ramping)
		sendSabertooth(address, 17, deadband)
except:
	print("motor controller setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)

# set up GPIOs
try:
	GPIO.setmode(GPIO.BOARD)
	#GPIO.setup(12, GPIO.OUT)
except:
	print("GPIO setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)

# begin server connection
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.bind(("", drivePort))
	serverSocket.listen(0)
	print("using serial port " + controller.name)
	print("DriveServer listening on port " + str(drivePort))
	while(True):
		(driveSocket, clientAddress) = serverSocket.accept()
		print("Connected to " + str(clientAddress[0]))
		while(True):
			data = driveSocket.recv(256)
			if(data == ""): # socket closing
				stopSabertooth()
				break
			else:
				parseCommand(data)
		print("Connection to " + str(clientAddress[0]) + " was closed")
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	stopSabertooth()
	stopSockets()
	GPIO.cleanup()
except socket.error as e:
	print(e.strerror)
	stopSabertooth()
	stopSockets()
	time.sleep(2)
	GPIO.cleanup()
	#subprocess.call("sudo reboot", shell = True)
except socket.timeout as e:
	print(e.strerror)
	stopSabertooth()
	stopSockets()
	time.sleep(2)
	GPIO.cleanup()
	#subprocess.call("sudo reboot", shell = True)
except:
	stopSabertooth()
	stopSockets()
	GPIO.cleanup()
	raise
