# Gyroscope/Accelerometer Library for the MPU6050 IMU on the Rover
# Based upon a true story: http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html

import smbus
import math
import os
import time

class IMU:
	def __init__(self):
		self.power_mgmt_1 = 0x6b
		self.power_mgmt_2 = 0x6c
		self.address = 0x68
		self.scale = 16384.0
		self.bus = smbus.SMBus(1)
		self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)
	
	def read_byte(self,adr):
		return bus.read_byte_data(address, adr)

	def read_word(self,adr):
		high = bus.read_byte_data(address, adr)
		low = bus.read_byte_data(address, adr+1)
		val = (high << 8) + low
		return val

	def read_word_2c(self,adr):
		val = read_word(adr)
		if (val >= 0x8000):
			return -((65535 - val) + 1)
		else:
			return val

	def dist(self,a,b):
		return math.sqrt((a*a)+(b*b))

	def get_y_rotation(self,x,y,z):
		radians = math.atan2(x, dist(y,z))
		return -math.degrees(radians)

	def get_x_rotation(self,x,y,z):
		radians = math.atan2(y, dist(x,z))
		return math.degrees(radians)
		
	def pitch(self):
		self.accel_xout_scaled = read_word_2c(0x3b)/self.scale
		self.accel_yout_scaled = read_word_2c(0x3d)/self.scale
		self.accel_zout_scaled = read_word_2c(0x3f)/self.scale
		return get_y_rotation(self.accel_xout_scaled, self.accel_yout_scaled, self.accel_zout_scaled)
	
	def roll(self):
		self.accel_xout_scaled = read_word_2c(0x3b)/self.scale
		self.accel_yout_scaled = read_word_2c(0x3d)/self.scale
		self.accel_zout_scaled = read_word_2c(0x3f)/self.scale
		return get_x_rotation(self.accel_xout_scaled, self.accel_yout_scaled, self.accel_zout_scaled)