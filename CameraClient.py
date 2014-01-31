
# A library to communicate with the rover's camera systems
# Added by Jordan

	# dependency list

import socket
import time


	# class definition

class CameraClient: # class to handle camera feeds	
	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port
		self.commandCameraStart = "#CS"
		self.commandCameraEnd = "#CE"

	def connect(self, retries):
		connected = False
		self.socket.settimeout(1.0)
		print("Connecting CameraClient to " + self.IP + ":" + str(self.port))
		for i in range (0, retries):
			try:
				self.socket.connect((self.IP, self.port))
				connected = True
				print("\tConnected to " + self.IP + ":" + str(self.port))
			except socket.error:
				print("\tretry " + str(i+1))
			if connected:
				return True
			else:
				time.sleep(1)
		print("\tCould not connect.")
		return False
	
	def startCamera(self):
		try:
			self.socket.send(self.commandCameraStart)
			return True
		except socket.error as e:
			print(e)
			return False
	
	def stopCamera(self):
		try:
			self.socket.send(self.commandCameraEnd)
			return True
		except socket.error as e:
			print(e)
			return False

	def test(self):
		try:
			self.socket.settimeout(0.1)
			self.socket.send("HELLO THAR")
			return True
		except socket.error as e:
			return False

