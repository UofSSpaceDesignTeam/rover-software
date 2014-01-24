#WORK IN PROGRESS!!!
#Proof of Concept to test sending control data over Sockets from an Xbox Controlled
#Based off Jordan's ControllerTest and my LEDCommsTest code.
#Written by Austin Shirley

import pygame
import Controller
import socket
import time

#Define Some variables
scaleFactor=1000
IP="192.168.1.104"
port=5001
deadzone=.1
deadzoneDelay=.1
  
def startSocket(): #sets up the socket
	sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	connected=False
	while(connected==False):
		print("Connecting...")
		try:
			sock.connect((IP,port))
			connected=True
		except:
			connected=False
	print("Connected to Pi")
	return sock

def getAxis(): #collects the position of the left x-axis
	LXAxis=Controller.getAxes()[0]
	LXRounded=round(LXAxis, 3) #Rounds the value to 3 decimal places
	print("LXRounded: " + str(LXRounded))
	return LXRounded
	
def getButtons(): #Collects button data (only one for now)
	Abutton=Controller.getButtons()[0]
	return Abutton

def makePacket(axisPos,button):  #builds a string of control data
	#Index characters for identifying data (subject to change)
	indexStrServo ="A"
	indexStrLED ="B"
	commandStr1 = "1,s" 
	commandStr2 = "\r\n"
	
	#Create the servo command from axis position.
	#Note that the motor must pause before changing direction
	#This action is defined in the function below
	print(axisPos)
	if(abs(axisPos) <= deadzone):
		time.sleep(deadzoneDelay)
	accel=str(int(scaleFactor*axisPos))
	
	#Create the LED Command
	print(button)
	if(button==1):
		buttoncmd="001"
	elif(button==0):
		buttoncmd="000"
		
	#Complile long packet of all the collected data
	pack = indexStrServo + commandStr1 + accel + commandStr2 + indexStrLED + buttoncmd
	return pack

#Main Execution 

pygame.init()

try:
	Controller.init()
except Exception as e:
	print(e)
	exit(1)	

sock=startSocket()
time.sleep(1)

while(True):
	pygame.event.pump()
	print("Preparing packet")
	pack = makePacket(getAxis(), getButtons())
	print(pack)
	sock.sendall(pack)
	print("Packet sent")
	time.sleep(0.01)