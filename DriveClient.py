# A library to communicate with the rover's movement system.

import socket
import time
	
class DriveClient: # class for drive control

	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port
		self.commandOneStickData = "#D1"
		self.commandTwoStickData = "#D2"
		self.commandRoverStop = "#DS"
		self.commandGPSData = "#GD"

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

	def sendOneStickData(self, xAxis, yAxis, limit):
		xInt = int(xAxis * 127) + 127
		yInt = int(yAxis * 127) + 127
		try:
			self.socket.send(self.commandOneStickData + chr(xInt) + chr(yInt) + chr(limit))
			return True
		except socket.error as e:
			print(e.strerror)
			self.stopMotors()
			return False
	
	def sendTwoStickData(self, leftAxis, rightAxis):
		leftInt = int(leftAxis * 127) + 127
		rightInt = int(rightAxis * 127) + 127
		try:
			self.socket.send(self.commandTwoStickData + chr(leftInt) + chr(rightInt))
			return True
		except socket.error as e:
			print(e.strerror)
			self.stopMotors()
			return False	
	
	def stopMotors(self):
		try:
			self.socket.send(self.commandRoverStop)
			return True
		except socket.error as e:
			print(e.strerror)
			return False
	
	def test(self):
		try:
			self.socket.settimeout(0.05)
			self.socket.send("TST")
			return True
		except socket.error:
			return False

