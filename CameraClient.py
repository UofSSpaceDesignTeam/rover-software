# A library to communicate with the rover camera systems

# dependency list

import socket
import time
import sys

# class definition

class CameraClient: # class to handle camera feeds	
	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port
		self.commandCameraStart = "#CS"
		self.commandCameraEnd = "#CE"
		self.commandCameraPicture = "#CP"

	def connect(self, retries):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.settimeout(0.7)
		try:
			self.socket.connect((self.IP, self.port))
			return True
		except socket.error:
			pass
		for i in range (0, retries):
			time.sleep(0.25)
			try:
				self.socket.connect((self.IP, self.port))
				return True
			except socket.error:
				pass
		return False
	
	def startCamera(self):
		try:
			self.socket.send(self.commandCameraStart)
			return True
		except socket.error as e:
			#sys.stderr.write(e.strerror)
			return False
	
	def stopCamera(self):
		try:
			self.socket.send(self.commandCameraEnd)
			return True
		except socket.error as e:
			#sys.stderr.write(e.strerror)
			return False

	def takePicture(self):
		try:
			self.socket.send(self.commandCameraPicture)
			return True
		except socket.error as e:
			#sys.stderr.write(e.strerror)
			return False

	def getPicture(self): #todo: windows version
		cmd = ("scp " + str(self.IP) + ": /home/pi/pictures/* ./robotPictures/" )
		print(cmd)
		subprocess.call(cmd, shell = True)

	def test(self):
		try:
			self.socket.settimeout(0.05)
			self.socket.send("TST")
			return True
		except socket.error:
			return False

