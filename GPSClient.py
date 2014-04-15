# A library to communicate with the rover's movement system and GPS.

# dependency list

import socket
import time

# class definition
	
class GPSClient: # class for drive control

	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port

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

	def getPosition(self):
		try:
			self.socket.settimeout(0.3)
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

