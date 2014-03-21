# A script continuously run by the arm control pi.

# dependency list

import math
import socket
import time
import subprocess
import serial
from ServoDriver import *
import RPi.GPIO as GPIO # for hardware reset system

# global constants	

armPort = 3003
ramping = 10
scaleFactor = 0.55
address= 128

Lalpha = 371	# Lalpha, Lbeta, A, B are in mm, to the closes mm 
Lbeta = 104		
A = 121
B = 363
# global variables

emergency = False

# function definitions

def readADC

def getActuatorFeedback
	ActuatorPos1=readADC()
	ActuatorPos2=readADC()
	print "Actuator 1 Position: ", ActuatorPos1
	print "Actuator 2 Position: ", ActuatorPos2
	
def sendSabertooth(address, command, speed):
	checksum = int(address) + int(command) + int(speed) & 127
	controller.write(chr(int(address)))
	controller.write(chr(int(command)))
	controller.write(chr(int(speed)))
	controller.write(chr(int(checksum)))
	
def TranslateZ(speed)
	#angles in radians
	#need actuator positions and constants
	#Lalpha,Lbeta,thetaL, thetaE, A, B, alpha, Lgamma, Ldelta are constants
	#L1 and L2 are actuator lengths, theta1 and theta2 are angular positions of L1 and L2 respectively
	#Rh is the ratio theta1_dot/theta2_dot, theta1_dot and theta2_dot are derivatives of theta1 and theta2 
	theta1=math.acos((pow(Lalpha,2)+pow(Lbeta,2)-pow(L1,2))/(2*Lalpha*Lbeta))+thetaL+thetaE
	theta2=math.acos((pow(A,2)+pow(B,2)-pow(L2,2))/(2*A*B)) + alpha + math.pi/2
	Rh=-Lgamma*math.cos(theta1+theta2)/(Lgamma*math.cos(theta1)+Ldelta*math.cos(theta1+theta2))
	theta2_dot=speed/(Ldelta*math.cos(theta1+theta2)*(Rh+1)+Rh*Lgamma*math.cos(theta1))
	theta1_dot=Rh*theta2_dot
	L1p = (theta1_dot*Lgamma*Lbeta)/(L1) * math.sqrt( abs((1 - pow( ( (pow(Lgamma,2)+pow(Lbeta,2)-pow(L1,2))/(2*Lgamma*Lbeta)),2))))
	L2p  = (theta2_dot*Lgamma*B)/(L2) * math.sqrt( abs((1 - pow( ((pow(A,2)+pow(B,2)-pow(L2,2))/(2*A*B)),2))))
	if A1Speed<=0:
		sendSabertooth(address,5,A1Speed)
	else:
		sendSabertooth(address,4,A1Speed)
	if A2Speed<=0:
		sendSabertooth(address,1,A2Speed)
	else:
		sendSabertooth(address,0,A2Speed)
	
def TranslateIO(speed)
	#angles in radians
	#need actuator positions and constants
	#Lalpha,Lbeta,thetaL, thetaE, A, B, alpha, Lgamma, Ldelta are constants
	#L1 and L2 are actuator lengths, theta1 and theta2 are angular positions of L1 and L2 respectively
	#Rr is the ratio theta1_dot/theta2_dot, theta1_dot and theta2_dot are derivatives of theta1 and theta2 
	theta1=math.acos((pow(Lalpha,2)+pow(Lbeta,2)-pow(L1,2))/(2*Lalpha*Lbeta))+thetaL+thetaE
	theta2=math.acos((pow(A,2)+pow(B,2)-pow(L2,2))/(2*A*B)) + alpha + math.pi/2
	Rr=-Ldelta*math.sin(theta1+theta2)/(Ldelta*math.sin(theta1)+Lgamma*math.sin(theta1+theta2))
	theta2_dot=-speed/(Ldelta*math.sin(theta1+theta2)*(Rr+1)+Rr*Lgamma*math.sin(theta1))
	theta1_dot=Rr*theta2_dot
	L1p = (theta1_dot*Lgamma*Lbeta)/(L1) * math.sqrt( abs((1 - pow( ( (pow(Lgamma,2)+pow(Lbeta,2)-pow(L1,2))/(2*Lgamma*Lbeta)),2))))
	L2p  = (theta2_dot*Lgamma*B)/(L2) * math.sqrt( abs((1 - pow( ((pow(A,2)+pow(B,2)-pow(L2,2))/(2*A*B)),2))))
	if A1Speed<=0:
		sendSabertooth(address,5,A1Speed)
	else:
		sendSabertooth(address,4,A1Speed)
	if A2Speed<=0:
		sendSabertooth(address,1,A2Speed)
	else:
		sendSabertooth(address,0,A2Speed)
	
	
def testSetActuators(actuator1, actuator2):
	throttlel = (actutaor1 - 127) / 127.0  # range is now -1 to 1
	throttler = (actuator2 - 127) / 127.0

	# Math for Actuators
	leftSpeed = (actuator1) * 127
	rightSpeed = (actuator2) * 127
	
	leftSpeed = max(leftSpeed, -127)
	leftSpeed = min(leftSpeed, 127)
	rightSpeed = max(rightSpeed, -127)
	rightSpeed = min(rightSpeed, 127)
	
	getActuatorFeedback()
	
	# send forward / reverse commands to controllers
	if(leftSpeed >= 0):
		sendSabertooth(address, 0, leftSpeed)
	else:
		sendSabertooth(address, 1, -1 * leftSpeed)
	
	if(rightSpeed >= 0):
		sendSabertooth(address, 4, rightSpeed)
	else:
		sendSabertooth(address, 5, -1 * rightSpeed)
	
def parseCommand(command): # Parses Socket Data back to Axis positions
	global emergency
	if len(command) > 3:
		if command[0] == "#": # is valid
			if command[1] == "A":
				if command[2] == "B": # rotate base
					if emergency == False:
						pass
				elif command[2] == "L": # translate wrist joint "up/down"
					if emergency == False:
						Speed = int(ord(command[3]))*127
						Speed = max(Speed, -127)
						Speed = min(Speed, 127)
						TranslateZ(Speed)
				elif command[2] == "M": # translate wrist joint "in/out"
					if emergency == False:
						Speed = int(ord(command[3]))*127
						Speed = max(Speed, -127)
						Speed = min(Speed, 127)
						TranslateIO(Speed)
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
					sendSabertooth(0, 0)
					servoDriver.reset()
					print("emergency stop")
					emergency = True
				elif command[2] == "C":
					emergency = False
				elif command[2] == "T":	
					actuator1 = int(ord(command[3]))
					actuator2 = int(ord(command[4]))
					testSetActuators(actuator1, actuator2)
					
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

# set up Sabertooth
try:
	controller = serial.Serial("/dev/ttyAMA0", bytesize = 8, parity = 'N', stopbits = 1)
	controller.baudrate = 9600
	controller.timeout = 0.2
	sendSabertooth(address, 16, ramping)

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
				sendSabertooth(0, 0)
				break
			else:
				parseCommand(data)
		print("Connection to " + str(clientAddress[0]) + " was closed")
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	sendSabertooth(0, 0)
	stopSockets()
	GPIO.cleanup()
except socket.error as e:
	print(e.strerror)
	sendSabertooth(0, 0)
	stopSockets()
	GPIO.cleanup()
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)
except:
	sendSabertooth(0, 0)
	stopSockets()
	GPIO.cleanup()
	raise
