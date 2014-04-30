import socket
import time
import sys

class MastClient: # class for mast control

	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port
		self.commandMast = "#MC"
		self.commandRoverStop = "#MS"


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

	def sendData(self, xDpad, yDpad):
		xDpad = int(xDpad)
		yDpad = int(yDpad)
		try:
			self.socket.send(self.commandMast + chr(xDpad) + chr(yDpad))
			return True
		except socket.error as e:
			print(e.strerror)
			self.stopServos()
			return False
	
	def stopServos(self):
		try:
			self.socket.send(self.commandRoverStop)
		except:
			pass
	
	def test(self):
		try:
			self.socket.settimeout(0.05)
			self.socket.send("TST")
			return True
		except socket.error:
			return False
