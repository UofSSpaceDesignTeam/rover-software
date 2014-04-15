# A library to communicate with the rover's movement system and GPS.

import socket
import time
import struct
	
class GPSClient: # class for drive control

	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port

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

	def getPosition(self):
		try:
			self.socket.settimeout(0.3)
			packet = self.socket.recv(256)
			data = struct.unpack
			print(GPSData)
			# GPSList = GPSData.split(" ")
			# lat = int(GPSList[0])
			# lon = int(GPSList[1])
			# return (True, lat, lon)
		except exception as e:
			print(e.strerror)
			return None
	
	def test(self):
		try:
			self.socket.settimeout(0.05)
			self.socket.send("TST")
			return True
		except socket.error:
			return False

