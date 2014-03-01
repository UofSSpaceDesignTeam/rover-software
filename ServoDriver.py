# Class to run the servo driver board.

import smbus
import math
import time

class Servo:
	servoMin = 1200 # us
	servoMax = 1800
	pulsePerDegree = (servoMax - servoMin) / 180.0

	def __init__(self, driver, channel, min, max, default):
		self.driver = driver
		self.channel = channel
		self.min = min
		self.max = max
		self.scale = (max - min) / 180
		self.set(default)

	def set(self, position):
		position = max(position, 0)
		position = min(position, 180)
		position *= self.scale
		position += self.min
		self.driver.setServo(self.channel, position)

class ServoDriver:
	def __init__(self):
		self.pwm = PWM(0x40, debug = False)
		self.pwm.setPWMFreq(60)

	def setServo(self, channel, position):
		if(channel >= 0 and channel <= 15):
			self.pwm.setPWM(channel, 0, position)
		else:
			print("invalid servo channel")


class PWM:
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

  def __init__(self, address=0x40, debug=False):
    self.i2c = Adafruit_I2C(address)
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
    self.i2c.write8(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.i2c.write8(self.__LED0_ON_H+4*channel, on >> 8)
    self.i2c.write8(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.i2c.write8(self.__LED0_OFF_H+4*channel, off >> 8)



class Adafruit_I2C:
 
  def __init__(self, address, busnum=-1, debug=False):
    self.address = address
    self.bus = smbus.SMBus(1)
    self.debug = debug

  def reverseByteOrder(self, data):
    byteCount = len(hex(data)[2:].replace('L','')[::2])
    val = 0
    for i in range(byteCount):
      val = (val << 8) | (data & 0xff)
      data >>= 8
    return val

  def errMsg(self):
    print "Error accessing 0x%02X: Check your I2C address" % self.address
    return -1

  def write8(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    try:
      self.bus.write_byte_data(self.address, reg, value)
      if self.debug:
        print "I2C: Wrote 0x%02X to register 0x%02X" % (value, reg)
    except IOError, err:
      return self.errMsg()

  def write16(self, reg, value):
    "Writes a 16-bit value to the specified register/address pair"
    try:
      self.bus.write_word_data(self.address, reg, value)
      if self.debug:
        print ("I2C: Wrote 0x%02X to register pair 0x%02X,0x%02X" %
         (value, reg, reg+1))
    except IOError, err:
      return self.errMsg()

  def writeList(self, reg, list):
    "Writes an array of bytes using I2C format"
    try:
      if self.debug:
        print "I2C: Writing list to register 0x%02X:" % reg
        print list
      self.bus.write_i2c_block_data(self.address, reg, list)
    except IOError, err:
      return self.errMsg()

  def readList(self, reg, length):
    "Read a list of bytes from the I2C device"
    try:
      results = self.bus.read_i2c_block_data(self.address, reg, length)
      if self.debug:
        print ("I2C: Device 0x%02X returned the following from reg 0x%02X" %
         (self.address, reg))
        print results
      return results
    except IOError, err:
      return self.errMsg()

  def readU8(self, reg):
    "Read an unsigned byte from the I2C device"
    try:
      result = self.bus.read_byte_data(self.address, reg)
      if self.debug:
        print ("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" %
         (self.address, result & 0xFF, reg))
      return result
    except IOError, err:
      return self.errMsg()

  def readS8(self, reg):
    "Reads a signed byte from the I2C device"
    try:
      result = self.bus.read_byte_data(self.address, reg)
      if result > 127: result -= 256
      if self.debug:
        print ("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" %
         (self.address, result & 0xFF, reg))
      return result
    except IOError, err:
      return self.errMsg()

  def readU16(self, reg):
    "Reads an unsigned 16-bit value from the I2C device"
    try:
      result = self.bus.read_word_data(self.address,reg)
      if (self.debug):
        print "I2C: Device 0x%02X returned 0x%04X from reg 0x%02X" % (self.address, result & 0xFFFF, reg)
      return result
    except IOError, err:
      return self.errMsg()

  def readS16(self, reg):
    "Reads a signed 16-bit value from the I2C device"
    try:
      result = self.bus.read_word_data(self.address,reg)
      if (self.debug):
        print "I2C: Device 0x%02X returned 0x%04X from reg 0x%02X" % (self.address, result & 0xFFFF, reg)
      return result
    except IOError, err:
      return self.errMsg()

if __name__ == '__main__':
  try:
    bus = Adafruit_I2C(address=0)
    print "Default I2C bus is accessible"
  except:
    print "Error accessing default I2C bus"

