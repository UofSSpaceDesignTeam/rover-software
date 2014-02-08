# Used on the Pi to control drive motors
# Added by Austin Shirley

## To Do: Reduce shut-down delay and implement skid steer


import socket
import time
import subprocess
import serial


# Motor Control Variables
scaleFactor = 0.55
deadband=0
ramping=20
emergency = False


# Serial Variables
sBaudrate = 9600
sTimeout = 0.2


# Server Variables
drivePort = 3002
global clientAddress


# Sabertooth Variables
speedR = 0
speedL = 0
speed = 0


# Direction Commands (Right Forwards, Left Forwards etc)
RF = 0
RR = 1
LF = 4
LR = 5


# Motor Controller Addresses (m1: front wheels, m2 middle wheels etc)
m1 = 128
m2 = 129
m3 = 130


# Function Definitions
def beginSerial():
	drive = serial.Serial("/dev/ttyAMA0", bytesize=8, parity='N', stopbits=1, )
	drive.baudrate = sBaudrate
	drive.timeout = sTimeout
	drive.write
	print(drive.name)
	return drive
	
def beginSabertooth():
	address=127
	while(address <= 135):
		sendSabertooth(address,16,ramping)
		sendSabertooth(address,17,deadband)
		address +=1
	print("Motor controllers configured")
	return

def sendSabertooth(address, command, speed):
	checksum=int(address)+int(command)+int(speed) & 127
	drive.write(chr(int(address)))
	drive.write(chr(int(command)))
	drive.write(chr(int(speed)))
	drive.write(chr(int(checksum)))
	
def sabertoothKill():
	# Disclaimer, no extinct Tigers were harmed in the making of this function
	address=127
	while(address <= 135):
		sendSabertooth(address,RF,0)
		sendSabertooth(address,LF,0)
		address +=1
	print("All Motors Stopped (Hopefully)")

def driveCommand(xAxis,yAxis): # Converts controller position to Thrust commands
	xMid=128
	yMid=128
	drive=yAxis-128
	if(drive < 0):
		drive = drive + 1
	steer=xAxis-128
	if(steer < 0):
		steer = steer + 1
	
	#print(drive,steer)
	
	# Math for SkidSteer
	leftThrust=drive
	rightThrust=drive
	
	# Clean up command values in case of math error
	if(leftThrust > 127):
		leftThrust = 127
		print("Max Speed")
	if(leftThrust < -127):
		leftThrust = -127
		print("Max Speed")
	if(rightThrust > 127):
		rightThrust = 127
		print("Max Speed")
	if(rightThrust < -127):
		rightThrust = -127
		print("Max Speed")
	
	# Send Forward and Reverse Commands
	if(leftThrust >= 0):
		sendSabertooth(m1,LF,abs(leftThrust))
		ld="F"
	else:
		sendSabertooth(m1,LR,abs(leftThrust))
		ld="R"
	if(rightThrust >= 0):
		sendSabertooth(m1,RF,abs(rightThrust))
		rd="F"
	else:
		sendSabertooth(m1,RR,abs(rightThrust))
		rd="R"
		
	print(leftThrust,ld,rightThrust,rd)
	
def parseController(command): # Parses Socket Data back to Axis positions
	global emergency
	if len(command) > 2:
		if(command[0] == "#"): # is valid
			if(command[1] == "D"):
				if(command[2] == "D" and len(command) > 4): # Drive, Data
					xAxis = int(ord(command[3]))
					yAxis = int(ord(command[4]))
					print(xAxis,yAxis)
					driveCommand(xAxis,yAxis)
					if(emergency == True):
						if(xAxis == 128 and yAxis == 128):
							emergency = False
							print("Resuming Control")
						else:
							print("Please centre the Controller Axis to continue")
							sabertoothKill()
							time.sleep(2)
				elif(command[2] == "S"): # Drive, Stop
					print("emergency stop command received")
					sabertoothKill()
					time.sleep(2)

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

# Setup
drive=beginSerial()
beginSabertooth()

# Begin connection
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
	serverSocket.bind(("", drivePort))
	serverSocket.listen(0)
	print("DriveServer listening on port " + str(drivePort))
	while(True):
		(driveSocket, clientAddress) = serverSocket.accept()
		driveSocket.settimeout(1.15)
		print("Connected to " + str(clientAddress[0]))
		while(True):
			data = driveSocket.recv(256)
			if(data == ""): # socket closing
				sabertoothKill()
				break
			else:
				parseController(data)
		print("Connection to " + str(clientAddress[0]) + " was closed")

# Exception Handling
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	sabertoothKill()
	stopSockets()
except socket.error as e:
	print(e.strerror)
	sabertoothKill()
	stopSockets()
	time.sleep(2)
	subprocess.Popen("sudo python DriveServerSabertooth.py", shell = True) # restart on connection failure
except:
	sabertoothKill()
	stopSockets()
	raise