
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
		return
		# connected = False
		# while(not connected):
			# try:
				# print("Connecting to " + self.moveControlIP[0] + ":" + str(self.moveControlIP[1]))
				# self.socket.connect((self.IP, self.port))
				# connected = True
				# print("Connected to " + self.IP + ":" + str(self.port))
			# except socket.error:
				# print("Retrying...")


	def send(self, data):
		return
		# try:
			# self.socket.sendall(data)
			# return True
		# except socket.error as e:
			# print(e)
			# return false


	def retrieveFile(filename): # use scp to grab a file from the pi
		return


	def disconnectMove(self):
		self.socket.shutdown(socket.SHUT_RDWR)
		self.socket.close()

