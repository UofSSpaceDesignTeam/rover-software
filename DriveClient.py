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
		self.commandControlData = "#DD"
		self.commandRoverStop = "#DS"
		self.commandGPSData = "#GD"
		self.commandSkidSwitch = "#DM"

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

	def sendControlData(self, throttle, steering):
		try:
			self.socket.send(self.commandControlData + chr(steering) + chr(throttle))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			self.stopMotors()
			return False
	
	def sendSkidSwitch(self, skidmode):
		try:
			self.socket.send(self.commandSkidSwitch + chr(skidmode))
			return True
		except socket.error as e:
			return False
			
	
	def stopMotors(self):
		try:
			self.socket.send(self.commandRoverStop)
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False

	def getGPSData(self):
		try:
			self.socket.settimeout(0.3)
			self.socket.send(self.commandGPSData)
			GPSData = self.socket.recv(256)
			sys.stderr.write(GPSData)
			GPSList = GPSData.split(" ")
			lat = int(GPSList[0])
			lon = int(GPSList[1])
			return (True, lat, lon)
		except exception as e:
			sys.stderr.write(e.strerror)
			return (False, 0, 0)
	
	def test(self):
		try:
			self.socket.settimeout(0.05)
			self.socket.send("TST")
			return True
		except socket.error:
			return False

