# A script continuously run by each Pi with a camera attached.
# RUN WITH SUDO

	# dependencies
	
import subprocess
import socket
import time


	# constants

commandPort = 3000
videoPort = 3001


	# functions

def startCamera():
	command = "raspivid -b 500000 -n -ex fixedfps -fps 20 -t 0 -rot 180 -o - | nc " + clientAddress[0] + " " + str(videoPort)
	#print(command)
	subprocess.Popen(command, shell = True)

def stopCamera():
	command = "sudo killall nc; sudo killall raspivid"
	#print(command)
	try:
		subprocess.call(command, shell = True)
	except:
		pass

def parseCommand(command):
	if(len(command) > 2 and command[0] == "#"): # is valid
		if(command[1] == "C"):
			if(command[2] == "S"): #CS
				print("starting camera feed")
				startCamera()
			elif(command[2] == "E"): #CE
				print("stopping camera feed")
				stopCamera()
			elif(command[2]== "P"): #CP
				print("taking picture")
				takePicture()

def takePicture():
	command = "raspistill -t 1000 -rot 180 -o /home/pi/pictures/" + time.strftime("%m%d%H%M%S", time.localtime()) + ".jpg"
	print(command)
	subprocess.call(command, shell = True)

def stopSockets():
	try:
		commandSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass


	# main execution

global clientAddress
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
	serverSocket.bind(("", commandPort))
	serverSocket.listen(0)
	print("CameraServer port: " + str(commandPort))
	print("Video port: " + str(videoPort))
	while(True):
		(commandSocket, clientAddress) = serverSocket.accept()
		print("Connected to: " + str(clientAddress[0]))
		while(True):
			data = commandSocket.recv(256)
			if(data == ""): # socket closing
				stopCamera()
				break
			else:
				parseCommand(data)
		print("Connection to: " + str(clientAddress[0]) + " closed")
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	stopCamera()
	stopSockets()
except socket.error as e:
	print(e.strerror)
	stopCamera()
	stopSockets()
	time.sleep(5)
	#subprocess.call("sudo reboot", shell = True)
except:
	stopCamera()
	stopSockets()
	#subprocess.call("sudo reboot", shell = True)
	raise

