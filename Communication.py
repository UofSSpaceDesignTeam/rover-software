
# A library to communicate with the rover's various systems
# Added by Jordan

	# dependency list

import socket
import time


class DriveClient: # class for drive systems
	commandMotorSet = "# M S"
	commandMotorStop = "# M Q"
	commandGPSData = "# G D"

	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port

	def Connect(self, retries):
		connected = False
		self.socket.settimeout(1.0)
		print("Connecting DriveClient to " + self.IP + ":" + str(self.port))
		for i in range (0, retries):
			try:
				self.socket.connect((self.IP, self.port))
				connected = True
				print("Connected to " + self.IP + ":" + str(self.port))
			except socket.error:
				print("retry " + str(i+1))
			if connected:
				return True
			else:
				time.sleep(1)
		return False

	def setMotors(self, leftMotor, rightMotor):
		try:
			self.socket.send(commandMotorSet)
			self.socket.send(chr(leftMotor))
			self.socket.send(chr(rightMotor))
			return True
		except socket.error as e:
			print(e)
			self.stopMotors()
			return False

	def stopMotors(self):
		try:
			self.socket.send(commandMotorStop)
			return True
		except socket.error as e:
			print(e)
			return False

	def getGPSData(self):
		try:
			self.socket.settimeout(0.3)
			self.socket.send(CommandGPSData)
			GPSData = self.socket.recv(256)
			print(GPSData)
			GPSList = GPSData.split(" ")
			lat = int(GPSList[0])
			lon = int(GPSList[1])
			return (True, lat, lon)
		except exception as e:
			print(e)
			return (False, 0, 0)



class ArmClient: # class for arm systems
	commandPanBase = "# A B" # spin base left / right
	commandLiftWrist = "# A L" # translate wrist joint up/down
	commandTiltWrist = "# A W" # rotate wrist joint up/down
	commandPanHand = "# A P" # move gripper left/right
	commandTwistHand = "# A H" # twist gripper cw/ccw
	commandGripper = "# A G" # open or close gripper
	
	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port

	def Connect(self, retries):
		connected = False
		self.socket.settimeout(1.0)
		print("Connecting ArmClient to " + self.IP + ":" + str(self.port))
		for i in range (0, retries):
			try:
				self.socket.connect((self.IP, self.port))
				connected = True
				print("Connected to " + self.IP + ":" + str(self.port))
			except socket.error:
				print("retry " + str(i+1))
			if connected:
				return True
			else:
				time.sleep(1)
		return False

	def panBase(self, speed):
		try:
			self.socket.send(commandPanBase)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False

	def LiftWrist(self, speed):
		try:
			self.socket.send(commandLiftWrist)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False

	def TiltWrist(self, speed):
		try:
			self.socket.send(commandTiltWrist)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False

	def PanHand(self, speed):
		try:
			self.socket.send(commandPanHand)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False

	def TwistHand(self, speed):
		try:
			self.socket.send(commandTwistHand)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False

	def Gripper(self, speed):
		try:
			self.socket.send(commandGripper)
			self.socket.send(chr(speed))
			return True
		except socket.error as e:
			print(e)
			return False



class CameraClient: # class to handle camera feeds
	commandCameraStart = "# C S"
	commandCameraEnd = "# C E"
	
	def __init__(self, IP, port):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.IP = IP
		self.port = port

	def Connect(self, retries):
		connected = False
		self.socket.settimeout(1.0)
		print("Connecting ArmClient to " + self.IP + ":" + str(self.port))
		for i in range (0, retries):
			try:
				self.socket.connect((self.IP, self.port))
				connected = True
				print("Connected to " + self.IP + ":" + str(self.port))
			except socket.error:
				print("retry " + str(i+1))
			if connected:
				return True
			else:
				time.sleep(1)
		return False
	
	def startCamera(self):
		try:
			self.socket.send(commandCameraStart)
			return True
		except socket.error as e:
			print(e)
			return False
	
	def stopCamera(self):
		try:
			self.socket.send(commandCameraEnd)
			return True
		except socket.error as e:
			print(e)
			return False

