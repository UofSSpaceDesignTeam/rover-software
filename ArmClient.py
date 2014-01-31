
# A library to communicate with the rover's arm control system
# Added by Jordan

	# dependency list

import socket
import time


	# class definition
	
class ArmClient: # class for arm systems
	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port
		self.commandPanBase = "#AB" # spin base left / right
		self.commandLiftWrist = "#AL" # translate wrist joint up/down
		self.commandTiltWrist = "#AW" # rotate wrist joint up/down
		self.commandPanHand = "#AP" # move gripper left/right
		self.commandTwistHand = "#AH" # twist gripper cw/ccw
		self.commandGripper = "#AG" # open or close gripper
		self.commandArmStop = "#AS" # stop all actuators
	
	def connect(self, retries):
		connected = False
		self.socket.settimeout(1.0)
		print("Connecting ArmClient to " + self.IP + ":" + str(self.port))
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
		print("\tCould not connect.")
		return False
	
	def panBase(self, speed):
		try:
			self.socket.send(self.commandPanBase)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False
	
	def liftWrist(self, speed):
		try:
			self.socket.send(self.commandLiftWrist)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False
	
	def tiltWrist(self, speed):
		try:
			self.socket.send(self.commandTiltWrist)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False
	
	def panHand(self, speed):
		try:
			self.socket.send(self.commandPanHand)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False
	
	def twistHand(self, speed):
		try:
			self.socket.send(self.commandTwistHand)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False
	
	def gripper(self, speed):
		try:
			self.socket.send(self.commandGripper)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False

	def stopMotors(self):
		try:
			self.socket.send(self.commandArmStop)
			return True
		except socket.error as e:
			print(e)
			return False

	def test(self):
		try:
			self.socket.settimeout(0.1)
			self.socket.send("HELLO THAR")
			return True
		except socket.error as e:
			return False

