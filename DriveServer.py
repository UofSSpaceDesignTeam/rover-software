# A script continuously run by the drive control Pi.

# dependency list

import socket
import time
import subprocess
import serial
import RPi.GPIO as GPIO # for hardware reset system

# global constants	

drivePort = 3002
ramping = 15
commandRF = 4
commandRR = 5
commandLF = 0
commandLR = 1

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
	sendSabertooth(129, commandRF, 0)
	sendSabertooth(129, commandLF, 0)

def setMotors(leftSpeed, rightSpeed):
	if abs(leftSpeed) > 127 or abs(rightSpeed) > 127:
		print("Motor value out of range.")
		stopSabertooth()
		return
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
					if command[2] == "1" and len(command) > 4: # one stick drive
						xChar = int(ord(command[3]))
						yChar = int(ord(command[4]))
						if emergency: # e-stop activated
							if xChar == 127 and yChar == 127:
								emergency = False
							else:
								print("Check controller connection and release joystick.")
								stopSabertooth()
								time.sleep(2)
						else: # not e-stopped
							leftSpeed = yChar + xChar - 254
							rightSpeed = yChar - xChar
							if max(abs(leftSpeed), abs(rightSpeed)) > 127:
								scaleFactor = 126.5 / max(abs(leftSpeed), abs(rightSpeed))
							else:
								scaleFactor = 1
							leftSpeed *= scaleFactor
							rightSpeed *= scaleFactor
							setMotors(leftSpeed, rightSpeed)
					elif command[2] == "2" and len(command) > 4: # two stick drive
						leftSpeed = int(ord(command[3])) - 127
						rightSpeed = int(ord(command[4])) - 127
						if emergency: # e-stop activated
							if(leftSpeed == 0 and rightSpeed == 0):
								emergency = False
							else:
								print("Check controller connection and release joystick.")
								stopSabertooth()
								time.sleep(2)
						else: # not e-stopped
							setMotors(leftSpeed, rightSpeed)
					elif command[2] == "S": # Stop
						stopSabertooth()
						print("emergency stop")
						emergency = True
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
	sendSabertooth(129, 16, ramping)
except:
	print("motor controller setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)

# set up GPIOs
# try:
	# GPIO.setmode(GPIO.BOARD)
	# GPIO.setup(12, GPIO.OUT)
# except:
	# print("GPIO setup failed!")
	# time.sleep(2)
	# raise
	# subprocess.call("sudo reboot", shell = True)

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

