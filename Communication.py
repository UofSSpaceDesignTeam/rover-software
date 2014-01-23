
# A library to handle IP connections to the rover
# Added by Dylan

	# dependency list

import socket
import time


	# class definition

class Communication:
	commandCameraStart = "# C S"
	commandCameraDisconnect = "# C C"
	commandTakePicture = ""
	commandControllerData = ""
	commandStopRover = ""

	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port


	def Connect(self, retries):
		connected = False
		print("Connecting to " + self.IP + ":" + str(self.port))
		for i in range (0, retries):
			try:
				self.socket.connect((self.IP, self.port))
				connected = True
				print("Connected to " + self.IP + ":" + str(self.port))
			except socket.error:
				print("retry " + str(i+1))
			if connected:
				return True
			else:
				time.sleep(1)
		return False
			


	def send(self, data):
		try:
			print(data)
			self.socket.sendall(data)
			return True
		except socket.error as e:
			print(e)
			return False


	def retrieveFile(filename): # use scp to grab a file from the pi
		return


	def disconnectCam(self):
		self.socket.send(commandCameraDisconnect)
		
	def connectCam(self):
		self.socket.send(commandCameraStart)

