# A library to communicate with the rover's movement system.

# dependency list

import socket
import time
import sys

# class definition
	
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
		self.socket.settimeout(1.0)
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

	def sendOneStickData(self, xAxis, yAxis):
		xChar = xAxis * 127 + 127
		yChar = yAxis * 127 + 127
		try:
			self.socket.send(self.commandOneStickData + chr(xChar) + chr(yChar))
			return True
		except socket.error as e:
			print(e.strerror)
			self.stopMotors()
			return False
	
	def sendTwoStickData(self, leftAxis, rightAxis):
		leftChar = axis1 * 127 + 127
		rightChar = axis2 * 127 + 127
		try:
			self.socket.send(self.commandTwoStickData + chr(leftChar) + chr(rightChar))
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

	def getGPSData(self):
		try:
			self.socket.settimeout(0.3)
			self.socket.send(self.commandGPSData)
			GPSData = self.socket.recv(256)
			print(GPSData)
			GPSList = GPSData.split(" ")
			lat = int(GPSList[0])
			lon = int(GPSList[1])
			return (True, lat, lon)
		except exception as e:
			print(e.strerror)
			return (False, 0, 0)
	
	def test(self):
		try:
			self.socket.settimeout(0.05)
			self.socket.send("TST")
			return True
		except socket.error:
			return False

