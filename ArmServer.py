# A script continuously run by the arm control pi.

# dependency list


import math
import socket
import time
import subprocess
import serial
from ServoDriver import *
from ADS1x15 import ADS1x15
from Adafruit_I2C import Adafruit_I2C
import RPi.GPIO as GPIO # for hardware reset system

# global constants	

armPort = 3003
ramping = 10
scaleFactor = 0.55
address= 128
gripperOpen = False

L1 = 300
L2 = 300

#arm constants
Lalpha = 371.29	# Lalpha, Lbeta, LA, LB, Lnu, Lmu are in mm, to the closes mm 
Lbeta = 104.88
Lnu = 56.83 	
Lmu = 369.37
LA = 121.12
LB = 363
thetaL = 0.2145 # in radians
thetaE = 0.8111
thetaA = 1.073
Ldelta = 108.16
Lgamma = 407.4 
ArmDeadband = 0

#actuator parameters
ActuatorFullIn = 292.354	#lengths again in mm
ActuatorFullOut = 444.754

Actuator1FullInRaw = 243
Actuator1FullOutRaw = 4749

Actuator2FullInRaw = 1890
Actuator2FullOutRaw = 3081

adc = ADS1x15(0x48)

# global variables

emergency = False

# function definitions

def readActuator1():
	#reads adc and maps the result to the current physical length of the actuator	
	result = adc.readADCSingleEnded(1)
	#map the result to the range 0->1
	result = (result - Actuator1FullInRaw) / (Actuator1FullOutRaw - Actuator1FullInRaw)
	#now map to the range fullIn -> fullOut
	result=result * (ActuatorFullOut - ActuatorFullIn) + ActuatorFullIn
	#result is now in mm's
	return result

def readActuator2():
	#reads the adc and maps the result to the current physical length of the actuator
	result = adc.readADCSingleEnded(2)
	#map the result to the range 0->1
	result = (result - Actuator2FullInRaw) / (Actuator2FullOutRaw - Actuator2FullInRaw)
	#now map to the range fullIn -> fullOut
	result=result * (ActuatorFullOut - ActuatorFullIn) + ActuatorFullIn
	#result is now in mm's
	return result
	
def sendSabertooth(address, command, speed):
	checksum = int(address) + int(command) + int(speed) & 127
	controller.write(chr(int(address)))
	controller.write(chr(int(command)))
	controller.write(chr(int(speed)))
	controller.write(chr(int(checksum)))
	
def TranslateZ(speed):
	#TODO: speed is not in the proper units yet

	#calculates and sends speeds of linear actuators for end effector to move up or down at input speed
	#angles in radians
	#Lalpha, Lbeta, thetaL, thetaE, LA, LB, thetaA, Lnu, Lmu are constants
	#L1 and L2 are actuator lengths, theta1 and theta2 are angular positions of L1 and L2 respectively
	#theta1 is arm angle of elevation, theta2 is elbow angle
	#Rh is the ratio theta1_dot/theta2_dot, theta1_dot and theta2_dot are derivatives of theta1 and theta2 
	#L1p and L2p are speeds of the linear actuators
	global L1
	global L2
	try:
		L2 = readActuator2()
		time.sleep(0.01)
		L1 = readActuator1()
	except:
		pass
	temp = (pow(Lalpha,2) + pow(Lbeta,2) - pow(L1,2)) / (2 * Lalpha * Lbeta)
	#to avoid math domain errors
	temp = max(temp,-1)
	temp = min(temp,1)

	#angles calculated from linear actuator lengths
	theta1 = math.acos(temp) + thetaL + thetaE - math.pi/2
	
	temp = (pow(LA,2) + pow(LB,2) - pow(L2,2)) / (2 * LA * LB)
	temp = max(temp,-1)
	temp = min(temp,1)
	theta2 = math.acos(temp) + thetaA

    #ratio to keep radius of extension constant
	Rr = (Lgamma*math.sin(theta1) - Ldelta*math.sin(theta1+theta2)) / (Ldelta*math.sin(theta1+theta2))

    #angular velocities to keep height velocity constant
	theta1_dot = speed /float (Lgamma*math.cos(theta1) - Ldelta*(Rh + 1)*math.cos(theta1+theta2))
	theta2_dot = Rr * theta1_dot

    #velocities of actuators to acheive angular velocities
	L1p = (theta1_dot * Lalpha * Lbeta)/(L1) * math.sqrt( abs((1 - pow( ( (pow(Lalpha,2) + pow(Lbeta,2) - pow(L1,2)) / (2 * Lalpha * Lbeta)),2))))
	L2p = (theta2_dot * LA * LB)/(L2) * math.sqrt( abs((1 - pow( ((pow(LA,2) + pow(LB,2) - pow(L2,2)) / (2 * LA * LB)),2))))
	#for debugging/testing
	print("In translateZ:")
	print("speed: ",speed)
	print("theta2: ", theta2)
	print("L1p; ", L1p)
	print("L2p: ",L2p)
	#deadband
	if abs(speed) <= ArmDeadband:
		L1p=0;
		L2p=0;
	#send the values to the actuators
	if L1p<=0:
		#constrain the range of data sent to sabertooth
		L1p=-L1p
		#actuator 1 gets stuck at low speeds, here is a simple correction. tweak values as necessary
		if abs(speed) > 0.2:
			L1p = L1p + 10
		L1p=max(0,L1p)
		L1p=min(127,L1p)
		sendSabertooth(address,1,L1p)
	else:
		#constrain the range of data sent to sabertooth
		#actuator 1 gets stuck at low speeds, here is a simple correction. tweak values as necessary
		if abs(speed) > 0.2:
			L1p = L1p + 10
		L1p=max(0,L1p)
		L1p=min(127,L1p)
		sendSabertooth(address,0,L1p)
	if L2p<=0:
		#constrain the range of data sent to sabertooth
		L2p=-L2p
		if abs(speed)>0.2:
			L2p=L2p+10
		L2p=max(0,L2p)
		L2p=min(127,L2p)
		sendSabertooth(address,5,L2p)
	else:
		#constrain the range of data sent to sabertooth
		if abs(speed)>0.2:
			L2p=L2p+10
		L2p=max(0,L2p)
		L2p=min(127,L2p)
		sendSabertooth(address,4,L2p)
		#for testing purposes
	if speed>0:
		print("Move Up")
	elif speed<0:
		print("Move Down")
	
def TranslateIO(speed):
	#calculates and sends speeds of linear actuators for end effector to move back or forth at speed
	#angles in radians
	#Lalpha, Lbeta, thetaL, thetaE, LA, LB, thetaA, Lnu, Lmu are constants
	#L1 and L2 are actuator lengths, theta1 and theta2 are angular positions of L1 and L2 respectively
	#theta1 is arm angle of elevation, theta2 is elbow angle
	#Rr is the ratio theta1_dot/theta2_dot, theta1_dot and theta2_dot are derivatives of theta1 and theta2 
	#L1p and L2p are speeds of the linear actuators

	L2 = readActuator2()
	L1 = readActuator1()

	temp = (pow(Lalpha,2) + pow(Lbeta,2) - pow(L1,2)) / (2 * Lalpha * Lbeta)
	#to avoid math domain errors
	temp = max(temp,-1)
	temp = min(temp,1)

	#angles calculated from linear actuator lengths
	theta1 = math.acos(temp) + thetaL + thetaE - math.pi/2
	temp = (pow(LA,2) + pow(LB,2) - pow(L2,2)) / (2 * LA * LB)
	temp = max(temp,-1)
	temp = min(temp,1)
	theta2 = math.acos(temp) + thetaA  

	#ratio to keep height constant  
	Rh = (Lgamma*math.cos(theta1) - Ldelta*math.cos(theta1+theta2)) / (Ldelta*math.cos(theta1+theta2))
	print("In translateIO")
	print("Rh: ",Rh)
	print("speed: ",speed)
	print("Ldelta: ", Ldelta)
	print("theta2: ", theta2)
	print("theta1: ", theta1)

    #angular velocities to keep radius velocity constant
	theta1_dot = (-speed) / (-Lgamma*math.sin(theta1) + Ldelta*(1+Rh)*math.sin(theta1+theta2))
	theta2_dot = Rh*theta1_dot

	print("theta1_dot: ", theta1_dot)
	print("theta2_dot: ",theta2_dot)
	L1p = (theta1_dot * Lalpha * Lbeta) / L1 * math.sqrt( abs((1 - pow( ( (pow(Lalpha,2) + pow(Lbeta,2) - pow(L1,2)) / (2 * Lalpha * Lbeta)),2))))
	L2p = (theta2_dot * LA * LB)/ L2 * math.sqrt( abs((1 - pow( ((pow(LA,2) + pow(LB,2) - pow(L2,2)) / (2 * LA * LB)),2))))
	print("L1p: ", L1p)
	print("L2p: ", L2p)
	if L1p<=0:
		#constrain the range of data sent to sabertooth
		L1p=-L1p
		L1p=max(0,L1p)
		L1p=min(127,L1p)
		sendSabertooth(address,1,L1p)
	else:
		#constrain the range of data sent to sabertooth
		L1p=max(0,L1p)
		L1p=min(127,L1p)
		sendSabertooth(address,0,L1p)
	if L2p<=0:
		#constrain the range of data sent to sabertooth
		L2p=-L2p
		L2p=max(0,L2p)
		L2p=min(127,L2p)
		sendSabertooth(address,5,L2p)
	else:
		#constrain the range of data sent to sabertooth
		L2p=max(0,L2p)
		L2p=min(127,L2p)
		sendSabertooth(address,4,L2p)
	#for testing purposes
	if speed<0:
		print("Move In")
	elif speed>0:
		print("Move Out")
		
def testSetActuators(actuator1, actuator2):
	actuator1 = (actuator1 - 127) / 127.0  # range is now -1 to 1
	actuator2 = (actuator2 - 127) / 127.0

	# Math for Actuators
	leftSpeed = (actuator1) * 127
	rightSpeed = (actuator2) * 127
	
	leftSpeed = max(leftSpeed, -127)
	leftSpeed = min(leftSpeed, 127)
	rightSpeed = max(rightSpeed, -127)
	rightSpeed = min(rightSpeed, 127)
	
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
						basePan.setAbsolute(int(ord(command[3])))
						if int(ord(command[3])) != 127: # stick not centered
							GPIO.output(12,False) # on
						else:
							GPIO.output(12,True) # off
						print(str(basePan.currentPosition))
				elif command[2] == "L": # translate wrist joint "up/down"
					if emergency == False:
						Speed = int(ord(command[3]))
						if Speed != 127:
							Speed = float(Speed - 127)/127	#range is now -1 to 1
							Speed = Speed*50		#adjust as necessary
							TranslateZ(Speed)
						else:
							sendSabertooth(address,4,0)
							sendSabertooth(address,5,0)
							sendSabertooth(address,1,0)
							sendSabertooth(address,0,0)
				elif command[2] == "M": # translate wrist joint "in/out"
					if emergency == False:
						Speed = int(ord(command[3]))
						if Speed != 127:
							Speed = float(Speed - 127)/127	#range is now -1 to 1
							Speed = Speed*20		#adjust as necessary
							TranslateIO(Speed)
						
				elif command[2] == "W": # rotate wrist joint up/down
					if emergency == False:
						print(int(ord(command[3])))
						wristTilt.setRelative(int(ord(command[3])))
						print("wristTilt")
				elif command[2] == "P": # pan gripper left/right
					if emergency == False:	
						wristPan.setRelative(int(ord(command[3])))
				elif command[2] == "H": # twist gripper cw/ccw
					if emergency == False:
						wristTwist.setRelative(int(ord(command[3])))
				elif command[2] == "G": # open or close gripper
					if emergency == False:
						if gripperOpen == True:
							servoDriver.setServo(6,1500)
							servoDriver.setServo(7,1700)
							gripperOpen = False
							print("gripper closed")
						else:
							servoDriver.setServo(6,1200)
							servoDriver.setServo(7,2000)
							gripperOpen = True
							print("gripper open")
				elif command[2] == "S": # stop all actuators
					sendSabertooth(address,0, 0)
					servoDriver.reset()
					print("emergency stop")
					emergency = True
				elif command[2] == "C": # cancel stop
					emergency = False
				elif command[2] == "T":	# controls both actuators individually 
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
	GPIO.setup(12,GPIO.OUT)
	GPIO.output(12,True)
except:
	print("GPIO setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)

# set up servo driver
try:
	servoDriver = ServoDriver()
	basePan = Servo(servoDriver, 8, 1000, 2200, 1596)
	wristPan = Servo(servoDriver, 11, 830, 2350, 1600)
	wristTilt = Servo(servoDriver, 10, 1000, 1700, 1370)
	wristTwist = Servo(servoDriver, 9, 830, 2350, 1600)
	servoDriver.setServo(6,1200)
	servoDriver.setServo(7,2000)
	#disconnect base servo power 
	GPIO.output(12,True)
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
	#main execution loop
	while(True):
		(armSocket, clientAddress) = serverSocket.accept()
		print("Connected to " + str(clientAddress[0]))
		while(True):
			data = armSocket.recv(256)
			if(data == ""): # socket closing
				sendSabertooth(address,0, 0)
				sendSabertooth(address,5, 0)
				break
			else:
				parseCommand(data)
		print("Connection to " + str(clientAddress[0]) + " was closed")
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	sendSabertooth(address,0, 0)
	sendSabertooth(address,5, 0)
	stopSockets()
	GPIO.cleanup()
except socket.error as e:
	print(e.strerror)
	sendSabertooth(address,0, 0)
	sendSabertooth(address,5, 0)
	stopSockets()
	GPIO.cleanup()
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)
except:
	sendSabertooth(address,0, 0)
	sendSabertooth(address,5, 0)
	stopSockets()
	GPIO.cleanup()
	raise
