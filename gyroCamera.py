from MPU6050 import IMU
from ServoDriver import *
import math
import time

class GyroCamera:
	def __init__(self, servoObject):
		try:
				self.imu = IMU()
		except:
			print("IMU setup failed!")
		self.currentPitch = 1300
		self.imuOldRoll = int(self.imu.roll())
		self.servoDriver = servoObject
		self.servoDriver.setServo(3, self.currentPitch)
	
	def angle2micros(self, angle):
		return int( 10 * angle )
		
	def angle2time(self, angle):
		return angle * 0.0133	#yet to be determined coefficient (was 0.01607083333)
	
	def setPitch(self, deltaPhi):
		if deltaPhi > 0:
			trav = self.angle2micros(deltaPhi)
			if self.currentPitch + trav < 2300:
				for x in range (0, trav):
					self.currentPitch = self.currentPitch + 1
					self.servoDriver.setServo(3, self.currentPitch)
				#self.adjustCameraPitchAngle(deltaPhi)
				
		elif deltaPhi < 0:
			trav = self.angle2micros(-1*deltaPhi)
			if self.currentPitch - trav > 780:
				for x in range (0, trav):
					self.currentPitch = self.currentPitch - 1
					self.servoDriver.setServo(3, self.currentPitch)
				#self.adjustCameraPitchAngle(deltaPhi)
				
	def setYaw(self, deltaTheta):

		if deltaTheta > 0:
			#print("Counter-Clockwise")
			waitTime = self.angle2time(deltaTheta)
			#print waitTime
			self.servoDriver.setServo(1, 1535)
			time.sleep(waitTime)
			self.servoDriver.setServo(1, 1550)
			#self.adjustCameraYawAngle(deltaTheta)
			
		elif deltaTheta < 0:
			#print("Clockwise")
			waitTime = self.angle2time(-1*deltaTheta)
			#print waitTime
			self.servoDriver.setServo(1, 1565)
			time.sleep(waitTime)
			self.servoDriver.setServo(1, 1550)
			#self.adjustCameraYawAngle(deltaTheta)
			
	def stableDriveMode(self, gyroEnable, p_dPad, y_dPad):
		if gyroEnable == 1:
			imuNewRoll = int(self.imu.roll())
			pTest = imuNewRoll - self.imuOldRoll
			
			if abs(pTest) > 2:
				#print("Change is in the IMU...")
				deltaCamPitch =  pTest
			else:
				deltaCamPitch = 0

			self.imuOldRoll = imuNewRoll
				
			# both are in DEGREES ( each d-Pad button push corresponds to 5 degrees )
			dCamPitch = deltaCamPitch + p_dPad * 5
			dCamYaw = y_dPad * -10

			# call f'ns to adjust physical camera pitch and yaw
			self.setPitch(dCamPitch)
			self.setYaw(dCamYaw)
			
		else:
			deltaCamPitch = 0
			# both are in DEGREES ( each d-Pad button push corresponds to 5 degrees )
			dCamPitch = deltaCamPitch + p_dPad * 5
			dCamYaw = y_dPad * -10

			# call f'ns to adjust physical camera pitch and yaw
			self.setPitch(dCamPitch)
			self.setYaw(dCamYaw)
