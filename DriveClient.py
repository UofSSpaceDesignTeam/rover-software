
# A library to communicate with the rover's movement system
# Added by Jordan

	# dependency list

import socket
import time


	# class definition
	
class DriveClient: # class for drive systems
	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port
		self.commandMotorSet = "#MS"
		self.commandMotorStop = "#MQ"
		self.commandGPSData = "#GD"

	def connect(self, retries):
		connected = False
		self.socket.settimeout(1.0)
		print("Connecting DriveClient to " + self.IP + ":" + str(self.port))
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
		return False

	def setMotors(self, leftMotor, rightMotor):
		try:
			self.socket.send(self.commandMotorSet)
			self.socket.send(chr(leftMotor))
			self.socket.send(chr(rightMotor))
			return True
		except socket.error as e:
			print(e)
			self.stopMotors()
			return False

	def stopMotors(self):
		try:
			self.socket.send(self.commandMotorStop)
			return True
		except socket.error as e:
			print(e)
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
			print(e)
			return (False, 0, 0)

