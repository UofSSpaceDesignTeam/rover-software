#WORK IN PROGRESS!!!
#Proof of Concept to test sending control data over Sockets from an Xbox Controlled
#Based off Jordan's ControllerTest and my LEDCommsTest code.
#Written by Austin Shirley

#TODO: Make an accompanying server on the Pi to listen for commands.

import pygame
import controller
import socket
import time

def startSocket(): #Set up the socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	IP = "192.168.1.104
	port = 5001
	sock.connect((IP,port))

def getAxis(): #collects the position of the left x-axis
	LXAxis = Controller.getAxes(0)
	LXRounded = round(LXAxis, 2)

def makePacket(axisPos):
	commandStr = '1,s/r/n"
	accel = #do some math here with axisPos!!
	pack = commandStr[:4] + accel + commandStr[4:]
	return pack

#Main Execution

pygame.init()
try:
	Controller.init()
except Exception as e:
	print(e)
	exit(1)	
startSocket()

while(True):
	pygame.event.pump()
	getAxis()
	sock.sendall(makePacket(LXRounded))
	time.sleep(0.25)