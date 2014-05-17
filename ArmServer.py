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
L1LowerLimit = 320
L2LowerLimit = 318

L1 = 350
L2 = 350

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
	#read the actuator's positions
		L2 = readActuator2()
		time.sleep(0.01)
		L1 = readActuator1()
	except:
		print("Lost connection to ADC")

	C1 = 1.0
	C2 = -(1.0 + 0.0007*pow((L1 - 400),2))
	
	L1p = C1*speed
	L2p = C2*speed

	#commented out for now
	
##	temp = float((pow(Lalpha,2) + pow(Lbeta,2) - pow(L1,2)) / (2 * Lalpha * Lbeta))
##	#to avoid math domain errors
##	temp = max(temp,-1)
##	temp = min(temp,1)
##
##	#angles calculated from linear actuator lengths
##	theta1 = math.acos(temp) + thetaL + thetaE - math.pi/2
##	
##	#to avoid math domain errors
##	temp = float((pow(LA,2) + pow(LB,2) - pow(L2,2)) / (2 * LA * LB))
##	temp = max(temp,-1)
##	temp = min(temp,1)
##
##	#angles calculate from actuator positions
##	theta2 = math.acos(temp) + thetaA
##
##        #ratio to keep radius of extension constant
##	Rr = float((Lgamma*math.sin(theta1) - Ldelta*math.sin(theta1+theta2)) / (Ldelta*math.sin(theta1+theta2)))
##
##        #angular velocities to keep height velocity constant
##	theta1_dot = float(speed / (Lgamma*math.cos(theta1) - Ldelta*(Rr + 1)*math.cos(theta1+theta2)))
##	theta2_dot = float(Rr * theta1_dot)
##
##        #velocities of actuators to acheive angular velocities
##	L1p = float((theta1_dot * Lalpha * Lbeta)/(L1) * math.sqrt( abs((1 - pow( ( (pow(Lalpha,2) + pow(Lbeta,2) - pow(L1,2)) / (2 * Lalpha * Lbeta)),2)))))
##	L2p = float((theta2_dot * LA * LB)/(L2) * math.sqrt( abs((1 - pow( ((pow(LA,2) + pow(LB,2) - pow(L2,2)) / (2 * LA * LB)),2)))))

	#for debugging/testing
	print("In translateZ:")
	print("Speed: ", speed)
	print("L1: ", L1)
	print("L2: ", L2)
	print("L1p; ", L1p)
	print("L2p: ",L2p)
	
	#send the values to the actuators
	if L1p<=0:
		#constrain the range of data sent to sabertooth
		L1p=-L1p
		L1p=max(0,L1p)
		L1p=min(127,L1p)
		#send the actuator speeds to the sabertooth
		sendSabertooth(address,1,L1p)
	else:
		#constrain the range of data sent to sabertooth
		#actuator 1 gets stuck at low speeds, here is a simple correction. tweak values as necessary
##		if abs(speed) < 0.2:
##			L1p = L1p + 10
		L1p=max(0,L1p)
		L1p=min(127,L1p)
		#send the actuator speeds to the sabertooth
		sendSabertooth(address,0,L1p)
	if L2p<=0:
		#constrain the range of data sent to sabertooth
		L2p=-L2p
##		if abs(speed) < 0.2:
##			L2p=L2p+10
		L2p=max(0,L2p)
		L2p=min(127,L2p)
		#send the actuator speeds to the sabertooth
		sendSabertooth(address,5,L2p)
	else:
		#constrain the range of data sent to sabertooth
##		if abs(speed)< 0.2:
##			L2p=L2p+10
		L2p=max(0,L2p)
		L2p=min(127,L2p)
		#send the actuator speeds to hte sabertooth
		sendSabertooth(address,4,L2p)
	
def TranslateIO(speed):
	#calculates and sends speeds of linear actuators for end effector to move back or forth at speed
	#angles in radians
	#Lalpha, Lbeta, thetaL, thetaE, LA, LB, thetaA, Lnu, Lmu are constants
	#L1 and L2 are actuator lengths, theta1 and theta2 are angular positions of L1 and L2 respectively
	#theta1 is arm angle of elevation, theta2 is elbow angle
	#Rr is the ratio theta1_dot/theta2_dot, theta1_dot and theta2_dot are derivatives of theta1 and theta2 
	#L1p and L2p are speeds of the linear actuators
	global L1
	global L2
	try:
	#read the actuator's positions
		L2 = readActuator2()
		time.sleep(0.01)
		L1 = readActuator1()
	except:
		print("Lost Connection to ADC")

	C1 = -1.25
	C2 = -1.0
        
	L1p = C1*speed
	L2p = C2*speed

##	temp = float((pow(Lalpha,2) + pow(Lbeta,2) - pow(L1,2)) / (2 * Lalpha * Lbeta))
##	#to avoid math domain errors
##	temp = max(temp,-1)
##	temp = min(temp,1)
##
##	#angles calculated from linear actuator lengths
##	theta1 = float(math.acos(temp) + thetaL + thetaE - math.pi/2)
##	temp = float((pow(LA,2) + pow(LB,2) - pow(L2,2)) / (2 * LA * LB))
##	#avoid math domain erros
##	temp = max(temp,-1)
##	temp = min(temp,1)
##
##	theta2 = float(math.acos(temp) + thetaA)  
##
##	#ratio to keep height constant  
##	Rh = float((Lgamma*math.cos(theta1) - Ldelta*math.cos(theta1+theta2)) / (Ldelta*math.cos(theta1+theta2)))
##
##        #angular velocities to keep radius velocity constant
##	theta1_dot = float((-speed) / (-Lgamma*math.sin(theta1) + Ldelta*(1+Rh)*math.sin(theta1+theta2)))
##	theta2_dot = float(Rh*theta1_dot)
##
##	#calculate actuator speeds from the angular velocities 
##	L1p = float((theta1_dot * Lalpha * Lbeta) / L1 * math.sqrt( abs((1 - pow( ( (pow(Lalpha,2) + pow(Lbeta,2) - pow(L1,2)) / (2 * Lalpha * Lbeta)),2)))))
##	L2p = float((theta2_dot * LA * LB)/ L2 * math.sqrt( abs((1 - pow( ((pow(LA,2) + pow(LB,2) - pow(L2,2)) / (2 * LA * LB)),2)))))

	print("In translateIO")
	print("Speed: ", speed)
	print("L1: ", L1)
	print("L2: ", L2)
	print("L1p; ", L1p)
	print("L2p: ",L2p)

	if L1p<=0:
		#constrain the range of data sent to sabertooth
		L1p=-L1p
		L1p=max(0,L1p)
		L1p=min(127,L1p)
		#send the actuator speeds to the sabertooth
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
		
def setActuators(actuator1, actuator2):
	#moves actuators independently
	actuator1 = (actuator1 - 127)   # range is now -127 to 127
	actuator2 = (actuator2 - 127) 

	# Math for Actuators
	leftSpeed = (actuator1)
	rightSpeed = (actuator2) 
	
	#constrain the range of data sent to the sabertooth
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
		sendSabertooth(address, 5, rightSpeed)
	else:
		sendSabertooth(address, 4, -1 * rightSpeed)
	
def parseCommand(command): # Parses Socket Data back to Axis positions
	global emergency
	if len(command) > 3:
		if command[0] == "#": # is valid
			if command[1] == "A":
				if command[2] == "B": # rotate base
					speed = 100 	#needs calibrating
					dir = int(ord(command[3]))
					if dir == 0:
						GPIO.output(12,False)	#disconnect servo power
					else:
						GPIO.output(12,True)	#connect servo power
					if dir == 1:
						servoDriver.setServo(8,1695 - speed)
					elif dir ==2:
						servoDriver.setServo(8,1695 + speed)
					else:
						servoDriver.setServo(8,1695)  
				elif command[2] == "L": # translate wrist joint 	
					Speed = int(ord(command[3]))
					print(Speed)
					if Speed != 127:	#if control sticks are off center, send new commands to actuators
						Speed = float(Speed - 127)/127	#range is now -1 to 1
						print("Speed: " + str(Speed))
						Speed = Speed*50		#adjust scaling as necessary
						TranslateIO(Speed)
					else:
						#stop actuators if control sticks are centered	
						sendSabertooth(address,4,0)
						sendSabertooth(address,0,0)
				elif command[2] == "M": # translate wrist joint 
					Speed = int(ord(command[3]))
					if Speed != 127:
						Speed = float(Speed - 127)/127 #range is now -1 to 1
						print("speed: " + str(Speed))
						Speed = Speed*50		#adjust scaling as necessary
						TranslateZ(Speed)
					else:
						#stop the actuators if control sticks are centered
						sendSabertooth(address,4,0)
						sendSabertooth(address,0,0)
						
				elif command[2] == "W": # rotate wrist joint up/down				
					#calculate the distance that needs to be traversed. 
					dist =  int(ord(command[3])) - 127
					if dist < 0:	
						dist = -dist
						#smooths the motion 
						for x in range(0,dist/3):	#dividing dist by 3 improves control significantly
							wristTilt.setRelative(-int(dist/20) + 127)
					else:
						#smooths the motion
						for x in range(0,dist/3):
							wristTilt.setRelative(int(dist/20) + 127)

				elif command[2] == "P": # pan gripper left/right					
					#calculate the distance that needs to be traversed
					dist = int(ord(command[3])) - 127
					speed = 2
					if dist < 0:
						dist = -dist
						if dist > 60:
							dist = 60
						#smooths the motion
						for x in range(0,dist/3):
							#wristPan.setRelative(127- dist/30)
							wristPan.setRelative(-speed + 127)
					else:
						#smooths the motion
						if dist > 60:
							dist = 60
						for x in range(0,dist/3):
							#wristPan.setRelative(127 + dist/30)
							wristPan.setRelative(speed + 127)
				elif command[2] == "H": # twist gripper cw/ccw			
					dir = int(ord(command[3]))
					speed = 2	#increase to rotate faster
					if dir == 1:
						#smooths the motion
						for x in range(0,50):
							wristTwist.setRelative(127 - speed)
					else:
						#smooths the motion
						for x in range(0,50):
							wristTwist.setRelative(127 + speed)	
				elif command[2] == "G": # open or close gripper
					temp = int(ord(command[3])) - 127
					#range of temp is now -127 to 127
					if temp >= 0:	#negative values correspond to the other trigger so ignore them
						temp = float(temp*800/127)
						#range of temp is now 0 to 800
						#right gripper:	
						#	open - 2000
						#	closed - 1200
						#left gripper:
						#	open - 1200
						#	closed - 2000
						#math for gripper position
						gripperRight = 2000 - int(temp)
						gripperLeft = int(temp) + 1200
						#update gripper position
						servoDriver.setServo(6,gripperLeft)
						servoDriver.setServo(7,gripperRight)
				elif command[2] == "S": # stop all actuators
					sendSabertooth(address,0, 0)
					sendSabertooth(address,4, 0)
					servoDriver.reset()
					print("emergency stop")
					emergency = True
				elif command[2] == "C": # cancel stop
					pass
				elif command[2] == "T":	# controls both actuators individually 
					global Length2
					global Length1
					#get speed commands from controller
					speed1 = int(ord(command[3]))
					speed2 = int(ord(command[4]))
					#try reading the adc
					try:
						Length2 = readActuator2()
						Length1 = readActuator1()
					except:
						pass
					#physical limits
					if (Length1 <= 340) & (speed1 < 127):
						speed1 = max(speed1, 65) 
					#lower limit for actuator 2
					if (Length2 <= L2LowerLimit) & (speed2 < 127):
						sendSabertooth(address,4,0)
					#lower limit for actuator 1
					elif (Length1 <= L1LowerLimit) & (speed1 < 127):
						sendSabertooth(address,0,0)
					else:
					#move the actuators
						setActuators(speed1, speed2)

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

# set up ADC
try:
	adc = ADS1x15(0x48)
except:
	print("adc setup failed")
	quit()

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
	GPIO.output(12,False)	# disconnect base servo power
except:
	print("GPIO setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)

# set up servo driver
try:
	servoDriver = ServoDriver()
	basePan = Servo(servoDriver, 8, 1000, 2200, 1695)
	wristPan = Servo(servoDriver, 11, 830, 2350, 1600)
	wristTilt = Servo(servoDriver, 10, 1000, 1700, 1370)
	wristTwist = Servo(servoDriver, 9, 830, 2350, 1600)
	servoDriver.setServo(6,1200)
	servoDriver.setServo(7,2000)
except:
	print("Servo setup failed!")
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)
	
# begin server connection
try:
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
				#todo: disconnect servo power 
				break
			else:
				parseCommand(data)
		print("Connection to " + str(clientAddress[0]) + " was closed")
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	sendSabertooth(address,0, 0)
	sendSabertooth(address,5, 0)
	#todo: disconnect servo power
	stopSockets()
	GPIO.cleanup()
except socket.error as e:
	print(e.strerror)
	sendSabertooth(address,0, 0)
	sendSabertooth(address,5, 0)
	#todo: disconnect servo power
	stopSockets()
	GPIO.cleanup()
	time.sleep(2)
	raise
	#subprocess.call("sudo reboot", shell = True)
except:
	print("error")
	sendSabertooth(address,0, 0)
	sendSabertooth(address,5, 0)
	#todo: disconnect servo power
	stopSockets()
	GPIO.cleanup()
	raise
