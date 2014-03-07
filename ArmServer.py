# A script continuously run by the arm control pi.

# dependency list

import socket
import time
import subprocess
import serial
from ServoDriver import *
import RPi.GPIO as GPIO # for hardware reset system

# global constants	

armPort = 3003
scaleFactor = 0.55

# global variables

emergency = False

# function definitions

def moveActuators(change1, change2): # sends movement data to kangaroo
	controller.write("1,pi" + str(int(speedActuator1 * scaleFactor)) + "\r\n"
	controller.write("2,pi" + str(int(speedActuator2 * scaleFactor)) + "\r\n")
	
def getFeedback():
	controller.write("1,getp\r\n")
	feedback1 = controller.readline()
	controller.write("2,getp\r\n")
	feedback2 = controller.readline()
	return (feedback1, feedback2)
	
def parseCommand(command): # Parses Socket Data back to Axis positions
	global emergency
	if len(command) > 3:
		if command[0] == "#": # is valid
			if command[1] == "A":
				if command[2] == "B": # rotate base
					if emergency == False:
						pass
				elif command[2] == "L": # translate wrist joint up/down
					if emergency == False:
						moveActuators(int(ord(command[3])) - 127, 0)
				elif command[2] == "M": # translate wrist joint in/out
					if emergency == False:
						pass
				elif command[2] == "W": # rotate wrist joint up/down
					if emergency == False:
						wristTilt.setRelative(int(ord(command[3])))
				elif command[2] == "P": # pan gripper left/right
					if emergency == False:	
						wristPan.setRelative(int(ord(command[3])))
				elif command[2] == "H": # twist gripper cw/ccw
					if emergency == False:
						wristTwist.setRelative(int(ord(command[3])))
				elif command[2] == "G": # open or close gripper
					if emergency == False:
						pass
				elif command[2] == "S": # stop all actuators
					moveActuators(0, 0)
					servoDriver.reset()
					print("emergency stop")
					emergency = True
				elif command[2] == "C":
					emergency = False
					
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

# set up kangaroo
try:
	controller = serial.Serial("/dev/ttyAMA0", bytesize = 8, parity = 'N', stopbits = 1)
	controller.baudrate = 9600
	controller.timeout = 0.2
	controller.write("1,start\r\n")
	controller.write("2,start\r\n")
except:
	print("kangaroo setup failed!")
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
	wristPan = Servo(servoDriver, 11, 830, 2350, 1600)
	wristTilt = Servo(servoDriver, 10, 1000, 1700, 1370)
	wristTwist = Servo(servoDriver, 9, 830, 2350, 1600)
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
				moveActuators(0, 0)
				break
			else:
				parseCommand(data)
		print("Connection to " + str(clientAddress[0]) + " was closed")
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	moveActuators(0, 0)
	stopSockets()
	GPIO.cleanup()
except socket.error as e:
	print(e.strerror)
	moveActuators(0, 0)
	stopSockets()
	GPIO.cleanup()
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)
except:
	moveActuators(0, 0)
	stopSockets()
	GPIO.cleanup()
	raise
