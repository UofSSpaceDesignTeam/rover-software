#include needed stuff here... like
from MPU6050 import IMU
import math
import time


class gyroCamera:

#	currentPitch = 1800
#	currentPitchAngle = 0.0
#	newPitchAngle = 0.0
#	currentYawAngle = 0.0
#	newYawAngle = 0.0
#	imuOldPitch = 0
#	imuOldRoll = 0

	def __init__(self, servos):
		self.currentPitch = 1300
		self.currentPitchAngle = 0.0
		self.newPitchAngle = 0.0
		self.currentYawAngle = 0.0
		self.newYawAngle = 0.0

		print("Starting IMU and servos...")
		try:
			imu = IMU()
		except:
			print("IMU setup failed!")
		self.imuOldPitch = int(imu.pitch())
		self.imuOldRoll = int(imu.roll())
		self.sevoDriver = servos
		self.servoDriver.setServo(3, self.currentPitch)
		time.sleep(2)	#wait for the camera to set
		print("done")
	
#converts pitch angle to microSeconds for servo...
#	based on 90deg servo
	def angle2micros(self, angle):
		return int( 11.1 * angle )

	def setPitch(self, newPhi):

		diff = newPhi - self.currentPitchAngle
		if diff < 0:
			if self.currentPitch > 800:
				trav = self.angle2micros(-1*diff)
				for x in range (0, trav):
					self.currentPitch = self.currentPitch - 1
					self.servoDriver.setServo(3, self.currentPitch)
		elif diff > 0:
			if self.currentPitch < 2300:
				trav = self.angle2micros(diff)
				for x in range (0, trav):
					self.currentPitch = self.currentPitch + 1
					self.servoDriver.setServo(3, self.currentPitch)

		self.currentPitchAngle = newPhi
	
	def angle2time(self, angle):
		return angle * 0.001	#yet to be determined coefficient

	def setYaw(self, newTheta):

		diff = newTheta - self.currentYawAngle
		if diff < 0:
			waitTime = self.angle2time(-1*diff)
			servoDriver.setServo(1, 1480)
			time.sleep(waitTime)
			self.servoDriver.setServo(1, 1500)
		elif diff > 0:
			waitTime = self.angle2time(diff)
			servoDriver.setServo(1, 1520)
			time.sleep(waitTime)
			self.servoDriver.setServo(1, 1500)

		self.currentYawAngle = newTheta


	def dist(self, a, b):
		return math.sqrt((a*a)+(b*b))

	def axisTransform(self, deltaPhi, deltaBeta):	#arguments are in DEGREES!!!

		#first calculate the previous vector (using RADIANS!!!)
		x = math.sin( math.radians(self.currentPitchAngle)) * math.cos( math.radians(self.currentYawAngle))
		y = math.sin( math.radians(self.currentPitchAngle)) * math.sin( math.radians(self.currentYawAngle))
		z = math.cos( math.radians(self.currentPitchAngle))
		#now for the various trig calc's to shorten upcoming step
		cosBeta = math.cos( math.radians(deltaBeta))
		sinBeta = math.sin( math.radians(deltaBeta))
		cosPhi = math.cos( math.radians(deltaPhi))
		sinPhi = math.sin( math.radians(deltaPhi))
		#now to calculate the new vector
		xprime = x*cosBeta + z*sinBeta
		yprime = x*sinBeta*sinPhi + y*cosPhi - z*sinPhi*cosBeta
		zprime = y*sinPhi - x*cosPhi*sinBeta + z*cosPhi*cosBeta
		#calculate, and convert to degrees, the new pitch and yaw angles
		thetaPrime = math.atan2(yprime, xprime)
		phiPrime = math.atan2( dist(xprime, yprime), zprime)
	
		self.newYawAngle = math.degrees(thetaPrime)
		self.newPitchAngle = math.degrees(phiPrime)

	def singleAdjust(self, gyroEnable, yButton, xButton):

		if gyroEnable == True:
			self.imuNewPitch = int(imu.pitch())
			self.imuNewRoll = int(imu.roll())
			pTest = abs(self.imuNewPitch - self.imuOldPitch)
			rTest = abs(self.imuNewRoll - self.imuOldRoll)
			if pTest > 0 or rTest > 0:
				self.axisTransform( self.imuNewRoll - self.imuOldRoll, self.imuNewPitch - self.imuOldPitch)
			else:
				self.newPitchAngle = self.currentPitchAngle
				self.newYawAngle = self.currentYawAngle
		else:
			self.newPitchAngle = self.currentPitchAngle
			self.newYawAngle = self.currentYawAngle
		#debugging prints
		print("Setting pitch to/from: %d, / %d" % (self.newPitchAngle - yButton, self.currentPitchAngle))
		print("Setting yaw to/from:   %d, / %d" % (self.newYawAngle + xButton, self.currentYawAngle))
		
		self.setPitch(self.newPitchAngle - yButton)	# -ve b/c of how phi is set up relative to servo time layout
		self.setYaw(self.newYawAngle + xButton)
		#update the current angles with the new ones
		self.currentPitchAngle = self.newPitchAngle - yButton
		self.currentYawAngle = self.newYawAngle + xButton