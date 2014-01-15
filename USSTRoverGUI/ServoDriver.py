#!/usr/bin/python

import time
import math
from I2C import I2C

class ServoDriver :
	i2c = None

	# Registers/etc.
	__SUBADR1            = 0x02
	__SUBADR2            = 0x03
	__SUBADR3            = 0x04
	__MODE1              = 0x00
	__PRESCALE           = 0xFE
	__LED0_ON_L          = 0x06
	__LED0_ON_H          = 0x07
	__LED0_OFF_L         = 0x08
	__LED0_OFF_H         = 0x09
	__ALLLED_ON_L        = 0xFA
	__ALLLED_ON_H        = 0xFB
	__ALLLED_OFF_L       = 0xFC
	__ALLLED_OFF_H       = 0xFD

	def __init__(self, address=0x46, debug=False):
		self.i2c = I2C(address,1)
		self.address = address
		self.debug = debug
		if (self.debug):
			print "Reseting PCA9685"
		self.i2c.write8(self.__MODE1, 0x00)

	def setPWMFreq(self, freq):
		"Sets the PWM frequency"
		prescaleval = 25000000.0    # 25MHz
		prescaleval /= 4096.0       # 12-bit
		prescaleval /= float(freq)
		prescaleval -= 1.0
		if (self.debug):
			print "Setting PWM frequency to %d Hz" % freq
			print "Estimated pre-scale: %d" % prescaleval
		prescale = math.floor(prescaleval + 0.5)
		if (self.debug):
			print "Final pre-scale: %d" % prescale
		oldmode = self.i2c.readU8(self.__MODE1);
		newmode = (oldmode & 0x7F) | 0x10             # sleep
		self.i2c.write8(self.__MODE1, newmode)        # go to sleep
		self.i2c.write8(self.__PRESCALE, int(math.floor(prescale)))
		self.i2c.write8(self.__MODE1, oldmode)
		time.sleep(0.005)
		self.i2c.write8(self.__MODE1, oldmode | 0x80)

	def setPWM(self, channel, on, off):
		"Sets a single PWM channel"
		self.i2c.write8(self.__LED0_ON_L+4*channel, on & 0xFF)
		self.i2c.write8(self.__LED0_ON_H+4*channel, on >> 8)
		self.i2c.write8(self.__LED0_OFF_L+4*channel, off & 0xFF)
		self.i2c.write8(self.__LED0_OFF_H+4*channel, off >> 8)

	def set(self, channel, angle):
		"Writes an angle to a servo"
		if(angle > 170):
			angle = 170
		elif(angle < 10):
			angle = 10
		self.setPWM(channel, 0, int(3.41*angle))
