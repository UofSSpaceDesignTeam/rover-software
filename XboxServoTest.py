#WORK IN PROGRESS!!!
#Proof of Concept to test sending control data over Sockets from an Xbox Controlled
#Based off Jordan's ControllerTest and my LEDCommsTest code.
#Written by Austin Shirley

#TODO: Make an accompanying server on the Pi to listen for commands.

import pygame
import Controller
import socket
import time

def startSocket(): #sets up the socket
	sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	IP="192.168.1.104"
	port=5001
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
	LXRounded=round(LXAxis, 2)
	print("LXRounded: " + str(LXRounded))
	return LXRounded

def makePacket(axisPos):  #builds a string of control data
	commandStr1="1,s" 
	commandStr2="\r\n"
	print(axisPos)
	accel=str(int(3000*axisPos))
	print(accel)
	pack=commandStr1 + accel + commandStr2
	return pack

#Main Execution

pygame.init()

try:
	Controller.init()
except Exception as e:
	print(e)
	exit(1)	

startSocket()
time.sleep(1)

while(True):
	pygame.event.pump()
	print("Preparing packet")
	print(makePacket(getAxis()))
	sock.send(makePacket(getAxis()))
	print("Packet sent")
	time.sleep(0.25)