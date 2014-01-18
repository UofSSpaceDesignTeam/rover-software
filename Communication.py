
# A library to handle IP connections to the rover
# Added by Dylan

	# dependency list

import socket
import time


	# class definition

class Communication:

	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port


	def connect(self):
		connected = False
		while(not connected):
			try:
				print("Connecting to " + self.moveControlIP[0] + ":" + str(self.moveControlIP[1]))
				self.socket.connect((self.IP, self.port))
				connected = True
				print("Connected to " + self.IP + ":" + str(self.port))
			except socket.error:
				print("Retrying...")


	def send(self, data):
		try:
			self.socket.sendall(data)
			return True
		except socket.error as e:
			print(e)
			return false


	def move(self, throttle, steering, pan, tilt):
		self.move.send('# C M' + " " + chr(throttle) + " " + chr(steering) +" " + chr(pan)+" " + chr(tilt))
		
	def startCamera(self):
		self.move.send('# C S')

	def disconnectMove(self):
		self.move.shutdown(socket.SHUT_RDWR)
		self.move.close()

	def disconnectArm(self):
		self.arm.shutdown(socket.SHUT_RDWR)
		self.arm.close()

