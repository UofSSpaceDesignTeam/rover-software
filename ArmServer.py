# A script continuously run by the arm control pi.
# Added by Jordan Kubica

import socket
import time
import subprocess
import serial
from ServoDriver import *
import RPi.GPIO as GPIO # for hardware reset system

	# constants	

armPort = 3003
scaleFactor = 0.55
deadband = 0
ramping = 20
commandIn1 = 0
commandOut1 = 1
commandIn2 = 4
commandOut2 = 5
controllerAddress = 128

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
	sendSabertooth(controllerAddress,commandIn1,0)
	sendSabertooth(controllerAddress,commandIn2,0)
	
def parseCommand(command): # Parses Socket Data back to Axis positions
	global emergency
	if len(command) > 3:
		if(command[0] == "#"): # is valid
			if(command[1] == "A"):
				# if(command[2] == "B": # rotate base
					# if(emergency == False)
						# rotateBase(int(ord(command[3])))
				# elif(command[2] == "L": # translate wrist joint up/down
					# if(emergency == False)
						# liftWrist(int(ord(command[3])))
				# elif(command[2] == "M": # translate wrist joint in/out
					# if(emergency == False)
						# moveWrist(int(ord(command[3])))
				# elif(command[2] == "W": # rotate wrist joint up/down
					# if(emergency == False)
						# rotateWrist(int(ord(command[3])))
				if command[2] == "P": # pan gripper left/right
					if emergency == False:	
						wristPan.set(int(ord(command[3])))
				# elif(command[2] == "H": # twist gripper cw/ccw
					# if(emergency == False)
						# twistGripper(int(ord(command[3])))
				# elif(command[2] == "G": # open or close gripper
					# if(emergency == False)
						# moveGripper(int(ord(command[3])))
				# elif(command[2] == "S": # stop all actuators
					# stopSabertooth()
					# print("emergency stop")
					# emergency = True
				# elif(command[2] == "C"):
					# emergency = False
	
					
def stopSockets(): # Stops sockets on error condition
	try:
		armSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass


### Main Program  ###

# set up motor controller
try:
	controller = serial.Serial("/dev/ttyAMA0", bytesize = 8, parity = 'N', stopbits = 1)
	controller.baudrate = 9600
	controller.timeout = 0.2
	sendSabertooth(controllerAddress, 16, ramping)
	sendSabertooth(controllerAddress, 17, deadband)
except:
	print("motor controller setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)

# set up GPIOs
try:
	GPIO.setmode(GPIO.BOARD)
	#GPIO.setup(7, GPIO.OUT)
except:
	print("GPIO setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)

# set up servo driver
try:
	servoDriver = ServoDriver()
	wristPan = Servo(servoDriver, 4, 1200, 1800, 90)
except:
	print("Servo setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)
	
# begin server connection
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.bind(("", armPort))
	serverSocket.listen(0)
	print("using serial port " + controller.name)
	print("ArmServer listening on port " + str(armPort))
	while(True):
		(armSocket, clientAddress) = serverSocket.accept()
		print("Connected to " + str(clientAddress[0]))
		while(True):
			data = armSocket.recv(256)
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
	GPIO.cleanup()
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)
except:
	stopSabertooth()
	stopSockets()
	GPIO.cleanup()
	raise
