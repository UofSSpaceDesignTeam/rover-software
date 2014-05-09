#include needed stuff here... like
from MPU6050 import IMU
from ServoDriver import *
import math
import time


# This utilizes a MPU6050 IMU and two servos (one 180: pitch
# and one continuous: yaw) to control the viewing angle of
# a camera (or anything that mounted to the two servos)
# In addition to the IMU, the function arguments
# yButton and xButton allow for external user control
# Also, there are two different ways of implementing this class:
#   -singleAdjust: full 3D camera pitch stabalization (not currently working)
#   -stableDriveMode: 2D stabalization (working but assumes camera is
#		facing forward ONLY)

class GyroCamera:

# __init__ (GyroCamera, ServoDriver)
# -initializes the IMU and applies the passed ServoDriver instance
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
		#print("Starting pitch is: %d" % self.imuOldPitch)
		#print("Starting roll is : %d" % self.imuOldRoll)
		self.servoDriver = servoObject
		self.servoDriver.setServo(3, self.currentPitch)
		time.sleep(0.25)	#wait for the camera to set
		print("Setup complete")


# angle2micros( Float(or Integer) )
# -converts pitch angle to microSeconds for servo...
# angle is assumed to be in DEGREES
	def angle2micros(self, angle):
		return int( 10 * angle )	#was 11.1 us/degrees
	
	def adjustCameraPitchAngle(self, delta):
		self.currentPitchAngle = self.currentPitchAngle + delta

# setPitch( Float(or Integer) )
# -changes the pitch servo position according to the passed change
# of angle, deltaPhi, which is in DEGREES
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

# angle2time( Float(or Integer) )
# -converts passed angle (degrees) to cts. servo rotation time (seconds)
# NOTE: conversion coefficient is not exact (becomes inaccurate over time)
	def angle2time(self, angle):
		return angle * 0.0133	#yet to be determined coefficient (was 0.01607083333)

# adjustCameraYawAngle( Float(or Integer) )
# -modifies current camera yaw angle by adding the passed change of angle, delta,
# which is in DEGREES
	def adjustCameraYawAngle(self, delta):
	
		self.yawMoveCount += 1	#debugging stuff
		
		newYaw = self.currentYawAngle + delta	#was just self.currentYawAngle += 84.699 / delta before
		
		if newYaw > 360:
			self.currentYawAngle = (newYaw) % 360
		elif newYaw < 0:
			self.currentYawAngle = 360 - abs( newYaw )
		else:
			self.currentYawAngle = newYaw

# setYaw( Float(or Integer) )
# -rotates the yaw servo position according to the passed change
# of angle, deltaTheta, which is in DEGREES
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

		print("Move count is: %d" % self.yawMoveCount)
		# print adjusted angles to verify correctness of operations
		print("New Pitch / New Yaw: %d / %d" % (self.currentPitchAngle, self.currentYawAngle))
		print("   ")
		


	def stableDriveMode(self, gyroEnable, p_dPad, y_dPad):

		if gyroEnable == True:
			imuNewPitch = int(self.imu.pitch())
			pTest = imuNewPitch - self.imuOldPitch
			
			if abs(pTest) > 2:
				print("Change is in the IMU...")
				deltaCamPitch = -1 * pTest
				self.imuOldPitch = imuNewPitch
			else:
				deltaCamPitch = 0
		else:
			deltaCamPitch = 0
			
		# both are in DEGREES ( each d-Pad button push corresponds to 5 degrees )
		dCamPitch = deltaCamPitch + p_dPad * 5
		dCamYaw = y_dPad * -10

		# call f'ns to adjust physical camera pitch and yaw
		self.setPitch(dCamPitch)
		self.setYaw(dCamYaw)