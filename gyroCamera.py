#include needed stuff here... like
from MPU6050 import IMU
from ServoDriver import *
import math
import time


class GyroCamera:

	def __init__(self, servoObject):
		self.currentPitch = 1300
		
		self.currentPitchAngle = 0.0
		self.currentYawAngle = 0.0

		self.yawMoveCount = 0
		
		print("Starting IMU and servos...")
		try:
			self.imu = IMU()
		except:
			print("IMU setup failed!")
		self.imuOldPitch = int(self.imu.pitch())
		self.imuOldRoll = int(self.imu.roll())
		print("Starting pitch is: %d" % self.imuOldPitch)
		print("Starting roll is : %d" % self.imuOldRoll)
		self.servoDriver = servoObject
		self.servoDriver.setServo(3, self.currentPitch)
		time.sleep(0.25)	#wait for the camera to set
		print("Setup complete")
	
#converts pitch angle to microSeconds for servo...
#	based on 90deg servo
	def angle2micros(self, angle):
		return int( 10 * angle )	#was 11.1 us/degrees
	
	def adjustCameraPitchAngle(self, delta):
		self.currentPitchAngle = self.currentPitchAngle + delta

	#deltaPhi -> degrees
	def setPitch(self, deltaPhi):

		if deltaPhi > 0:	#was <
			print("Higher...")
			trav = self.angle2micros(deltaPhi)
			if self.currentPitch + trav < 2300:
				for x in range (0, trav):
					self.currentPitch = self.currentPitch + 1
					self.servoDriver.setServo(3, self.currentPitch)
				self.adjustCameraPitchAngle(deltaPhi)
				
		elif deltaPhi < 0:	#was >
			print("Lower...")
			trav = self.angle2micros(-1*deltaPhi)
			if self.currentPitch - trav > 500:
				for x in range (0, trav):
					self.currentPitch = self.currentPitch - 1
					self.servoDriver.setServo(3, self.currentPitch)
				self.adjustCameraPitchAngle(deltaPhi)
	
	def angle2time(self, angle):
		return angle * 0.0133	#yet to be determined coefficient
	
	def adjustCameraYawAngle(self, delta):
	
		self.yawMoveCount += 1
		print("Move count is: %d" % self.yawMoveCount)
		
		if self.currentYawAngle + delta > 360:
			self.currentYawAngle = (self.currentYawAngle + delta) % 360
		elif self.currentYawAngle + delta < 0:
			self.currentYawAngle = 360 - abs( self.currentYawAngle + delta )
		else:
			self.currentYawAngle = self.currentYawAngle + delta

	#deltaTheta -> degrees
	def setYaw(self, deltaTheta):

		if deltaTheta > 0:
			print("Counter-Clockwise")
			waitTime = self.angle2time(deltaTheta)
			self.servoDriver.setServo(1, 1480)
			time.sleep(waitTime)
			self.servoDriver.setServo(1, 1500)
			self.adjustCameraYawAngle(deltaTheta)
			
		elif deltaTheta < 0:
			print("Clockwise")
			waitTime = self.angle2time(-1*deltaTheta)
			self.servoDriver.setServo(1, 1520)
			time.sleep(waitTime)
			self.servoDriver.setServo(1, 1500)
			self.adjustCameraYawAngle(deltaTheta)

	def calculateCamAdjust(self, dPitchIMU, dYawIMU):	#arguments and returns are in DEGREES!!!

		caseTest = int(self.currentYawAngle / 90)
		if caseTest == 0 or caseTest == 2:
			yWeight = (self.currentYawAngle % 90) / 90
			pWeight = 1  - yWeight
			return -1 * (-1*pWeight * dPitchIMU + yWeight * dYawIMU)
			
		elif caseTest == 1 or caseTest == 3:
			pWeight = (self.currentYawAngle % 90) / 90
			yWeight = 1 - pWeight
			return -1 * (-1*pWeight * dPitchIMU + yWeight * dYawIMU)
			
		else:
			print ("Failed to determine a case!!!")
			return 0

	def singleAdjust(self, gyroEnable, p_dPad, y_dPad):

		if gyroEnable == True:
			imuNewPitch = int(self.imu.pitch())
			imuNewRoll = int(self.imu.roll())
			pTest = imuNewPitch - self.imuOldPitch
			rTest = imuNewRoll - self.imuOldRoll
			#self.imuOldPitch = imuNewPitch
			#self.imuOldRoll = imuNewRoll
			#print("New/diff in Pitch: %d %d" % (self.imuNewPitch, pTest))
			#print("New/diff in Roll: %d %d" % (self.imuNewRoll, rTest))
			
			if abs(pTest) > 3 or abs(rTest) > 3:
				print("Change is in the IMU...")
				deltaCamPitch = self.calculateCamAdjust( pTest, rTest)
				self.imuOldPitch = imuNewPitch
				self.imuOldRoll = imuNewRoll
			else:
				deltaCamPitch = 0
		else:
			deltaCamPitch = 0
			
			
		# both are in DEGREES ( each d-Pad button push corresponds to 5 degrees )
		dCamPitch = deltaCamPitch + p_dPad * 5
		dCamYaw = y_dPad * -10
		
		#debugging prints
		print("Old Pitch / Yaw Angles: %d / %d" % (self.currentPitchAngle, self.currentYawAngle))
		print("Pitch / Yaw Changes   : %d / %d" % (dCamPitch, dCamYaw))
		
		# call f'ns to adjust physical camera pitch and yaw
		self.setPitch(dCamPitch)
		self.setYaw(dCamYaw)
		
		# print adjusted angles to verify correctness of operations
		print("New Pitch / New Yaw: %d / %d" % (self.currentPitchAngle, self.currentYawAngle))
		print("   ")