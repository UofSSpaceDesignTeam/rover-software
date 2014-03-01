# A script continuously run by the drive control Pi.
# Added by Austin Shirley

## To Do: Reduce shut-down delay and implement skid steer


import socket
import time
import subprocess
import serial
import RPi.GPIO as GPIO # for hardware reset system


	# constants	

drivePort = 3002
scaleFactor = 0.55
deadband = 0
ramping = 20
commandRF = 0
commandRR = 1
commandLF = 4
commandLR = 5
controllerAddress = [128, 129, 130]


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
		print "stopping"
		sendSabertooth(address,commandRF,0)
		sendSabertooth(address,commandLF,0)

def setMotors(xAxis, yAxis): # sends motor commands based on joystick position
	throttle = (yAxis - 127) / 127.0  # range is now -1 to 1
	steering = (xAxis - 127) / 127.0
	
	# Math for SkidSteer
	leftSpeed = 127*throttle
	rightSpeed = 127*throttle
	
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
	
def parseController(command): # Parses Socket Data back to Axis positions
	global emergency
	if len(command) > 2:
		if(command[0] == "#"): # is valid
			if(command[1] == "D"):
				if(command[2] == "D" and len(command) > 4): # Drive, Data
					xAxis = int(ord(command[3]))
					yAxis = int(ord(command[4]))
					print(xAxis, yAxis)
					setMotors(xAxis, yAxis)
					if(emergency == True):
						if(xAxis == 127 and yAxis == 127):
							emergency = False
						else:
							print("Check controller connection and release joystick.")
							stopSabertooth()
							time.sleep(2)
				elif(command[2] == "S"): # Stop
					stopSabertooth()
					print("emergency stop")
					emergency = True

def stopSockets(): # Stops sockets on error condition
	try:
		commandSocket.close()
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
	GPIO.setup(12, GPIO.OUT)
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
		driveSocket.settimeout(2)
		print("Connected to " + str(clientAddress[0]))
		while(True):
			data = driveSocket.recv(256)
			if(data == ""): # socket closing
				stopSabertooth()
				break
			else:
				parseController(data)
		print("Connection to " + str(clientAddress[0]) + " was closed")
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	stopSabertooth()
	stopSockets()
	GPIO.cleanup()
except socket.error as e:
	print(e.strerror)
	print("stopping")
	stopSabertooth()
	stopSockets()
	time.sleep(2)
	GPIO.cleanup()
	#subprocess.call("sudo reboot", shell = True)
except socket.timeout as e:
	print(e.strerror)
	print ("stopping")
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
