import socket
import time
class Communication:

	def __init__(self):
		self.arm = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.armIP = ('192.168.1.104',3000)
		self.move = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.moveControlIP = ('192.168.1.103',3000)


	def link(self, controlState):
		connected = False
		while(not connected):
			try:
				if controlState == 'move':
					print("Connecting to " + self.moveControlIP[0] + ":" + str(self.moveControlIP[1]))
					#self.move.connect(self.moveControlIP)
					connected = True
					print("Connected to " + self.moveControlIP[0] + ":" + str(self.moveControlIP[1]))
				else:
					print("Connecting to " + self.armIP[0] + ":" + str(self.armIP[1]))
					self.arm.connect(self.armIP)
					connected = True
					print("Connected to " + self.armIP[0] + ":" + str(self.armIP[1]))
			except socket.error:
				print("Retrying...")
		print("Connection Established")


	def move(self, throttle, steering, pan, tilt):
		self.move.send('CM' + chr(throttle) + chr(steering) + chr(pan) + chr(tilt))
		

	def disconnectMove(self):
		self.move.shutdown(socket.SHUT_RDWR)
		self.move.close()

	def disconnectArm(self):
		self.arm.shutdown(socket.SHUT_RDWR)
		self.arm.close()