# A script continuously run by the drive control Pi.

# dependency list

import socket
import time
import subprocess
import serial
import RPi.GPIO as GPIO # for hardware reset system

# global constants	

drivePort = 3002
ramping = 35 # 0.7 second
timeout = 10 # 1 second
commandRF = 4
commandRR = 5
commandLF = 0
commandLR = 1

# Function Definitions

def sendSabertooth(address, command, data):
	checksum = chr(address) + chr(command) + chr(data) & chr(127)
	controller.write(chr(address))
	controller.write(chr(command))
	controller.write(chr(data))
	controller.write(chr(checksum))
	
def stopSabertooth():
	sendSabertooth(129, commandRF, 0)
	sendSabertooth(129, commandLF, 0)

def setMotors(leftSpeed, rightSpeed):
	if leftSpeed > 127:
		leftSpeed = 127
	if leftSpeed < -127:
		leftSpeed = - 127
	if rightSpeed > 127:
		rightSpeed = 127
	if rightSpeed < -127:
		rightSpeed = -127
	# send forward / reverse commands to controllers
	if(leftSpeed >= 0):
		sendSabertooth(129, commandLF, leftSpeed)
	else:
		sendSabertooth(129, commandLR, -1 * leftSpeed)		
	if(rightSpeed >= 0):
		sendSabertooth(129, commandRF, rightSpeed)
	else:
		sendSabertooth(129, commandRR, -1 * rightSpeed)
			
def parseCommand(command): # parses and executes remote commands
	global emergency
	if command != None:
		if len(command) > 2:
			if command[0] == "#": # is valid
				if command[1] == "D":
					if command[2] == "1" and len(command) > 5: # one stick drive
						xChar = int(ord(command[3]))
						yChar = int(ord(command[4]))
						limit = int(ord(command[5]))
						leftSpeed = yChar + xChar - 254
						rightSpeed = yChar - xChar
						if max(abs(leftSpeed), abs(rightSpeed)) > limit:
							scaleFactor = float(limit) / max(abs(leftSpeed), abs(rightSpeed))
						else:
							scaleFactor = 1
						leftSpeed *= scaleFactor
						rightSpeed *= scaleFactor
						setMotors(leftSpeed, rightSpeed)
					elif command[2] == "2" and len(command) > 4: # two stick drive
						leftSpeed = int(ord(command[3])) - 127
						rightSpeed = int(ord(command[4])) - 127
						setMotors(leftSpeed, rightSpeed)
					elif command[2] == "S": # Stop
						stopSabertooth()
						print("motors stopped.")
	else: # command == none
		stopSabertooth()
					

def stopSockets():
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
except:
	print("motor controller setup failed!")
	raise
	#subprocess.call("sudo reboot", shell = True)

# set up GPIOs. Weirdness is required so all controllers are responsive.
try:
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(12, GPIO.OUT) # actually #18 on board, driving Nfet driving Pfet driving controller power
	GPIO.output(12, False) # turn off motor controllers to start
	time.sleep(1.0)
	GPIO.output(12, True) # turn on motor controllers first time
	time.sleep(0.5)
	sendSabertooth(129, 16, ramping)
	sendSabertooth(129, 14, timeout)
	time.sleep(0.5)
	GPIO.output(12, False) # turn off motor controllers again
	time.sleep(1.0)
	GPIO.output(12, True) # turn on motor controllers for realsies
	time.sleep(0.5)
	sendSabertooth(129, 16, ramping)
	sendSabertooth(129, 14, timeout)
	time.sleep(0.5)
except:
	print("GPIO setup failed!")
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
	#GPIO.cleanup()
except:
	stopSabertooth()
	stopSockets()
	#GPIO.cleanup()
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)

