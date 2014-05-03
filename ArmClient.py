
# A library to communicate with the rover's arm control system
# Added by Jordan

# dependency list

import socket
import time
import sys

# class definition

class ArmClient: # class for arm control
	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port
		self.commandPanBase = "#AB" # spin base left / right
		self.commandLiftWrist = "#AL" # translate wrist joint up/down
		self.commandMoveWrist = "#AM" # translate wrist joint in/out
		self.commandTiltWrist = "#AW" # rotate wrist joint up/down
		self.commandPanHand = "#AP" # move gripper left/right
		self.commandTwistHand = "#AH" # twist gripper cw/ccw
		self.commandGripper = "#AG" # open or close gripper
		self.commandArmStop = "#AS" # stop all actuators
		self.commandArmResume = "#AC" # cancel emergency stop
		self.commandActuators = "#AT" # controls actuator 1 directly 
		
	
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
	
	def panBase(self, speed):
		try:
			self.socket.send(self.commandPanBase + chr(speed))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False
	
	def liftWrist(self, speed):
		try:
			self.socket.send(self.commandLiftWrist + chr(speed))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False
			
	def moveWrist(self, speed):
		try:
			self.socket.send(self.commandMoveWrist + chr(speed))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False
	
	def tiltWrist(self, speed):
		try:
			self.socket.send(self.commandTiltWrist + chr(speed))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False
	
	def panHand(self, speed):
		try:
			self.socket.send(self.commandPanHand + chr(speed))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False
	
	def twistHand(self, speed):
		try:
			self.socket.send(self.commandTwistHand + chr(speed))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False
	
	def gripper(self, speed):
		try:
			self.socket.send(self.commandGripper)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False

	def stopMotors(self):
		try:
			self.socket.send(self.commandArmStop)
		except socket.error:
			pass

	def test(self):
		try:
			self.socket.settimeout(0.05)
			self.socket.send("TST")
			return True
		except socket.error:
			return False
	# Temporary for testing actuators		
	def temp_actuator1(self, actuator1, actuator2):
		try:
			self.socket.send("#AT" + chr(actuator1) + chr(actuator2))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			self.stopMotors()
			return False
			
	def actuators(self, actuator1, actuator2):
		try:
			self.socket.send(self.Actuators + chr(actuator1) + chr(actuator2))
			return True
		except socket.error as e:
			sys.stderr.write(e.strerror)
			return False		
	

