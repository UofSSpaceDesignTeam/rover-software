# A script continuously run by each Pi with a camera attached.

	# dependencies
	
import subprocess
import socket
import time
from threading import Thread


	# constants

commandPort = 3000
videoPort = 3001


	# functions

def startCamera(clientAddress):
	global p
	command = "raspivid -b 500000 -ex fixedfps -fps 20 -t 0 -rot 180 -o - | nc " + str(clientAddress) + " " + str(videoPort)
	p = subprocess.Popen(str(command),shell=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)

def stopCamera():
	p.stdout.flush
	p.stdout.close
	subprocess.call("killall nc; killall raspivid")

def parseCommand(command):
	if(len(command) > 2 and command[0] == "#"): # is valid
		if(command[1] == "C"):
			if(command[2] == "S"): #CS
				print("starting camera feed")
				startCamera()
			elif(datan[2] == "E"): #CE
				print("stopping camera feed")
				stopCamera()


	# main execution

global clientAddress
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
	serverSocket.bind(("", commandPort))
	serverSocket.listen(0)
	print("CameraServer listening on port " + str(commandPort))
	print("\tVideo will be sent on port " + str(videoPort))
	while(True):
		(commandSocket, clientAddress) = serverSocket.accept()
		print("Connected to " + str(clientAddress)) 
		while(True):
			data = commandSocket.recv(256)
			if(data == ""): # socket closing
				break
			else:
				parseCommand(data)
		print("Connection from " + str(clientAddress) + " closed")
except socket.error as e:
	print(e)
	time.Sleep(2)
	subprocess.call("python CameraServer.py") # restart on connection failure
