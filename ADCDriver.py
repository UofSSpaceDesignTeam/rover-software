# Class to run the ADC board.

import smbus
import time

class ADCDriver:
	i2c = None

	# IC Identifiers
	__IC_ADS1015 = 0x00

	# Pointer Register
	__ADS1015_REG_POINTER_MASK = 0x03
	__ADS1015_REG_POINTER_CONVERT = 0x00
	__ADS1015_REG_POINTER_CONFIG = 0x01
	__ADS1015_REG_POINTER_LOWTHRESH = 0x02
	__ADS1015_REG_POINTER_HITHRESH = 0x03

	# Config Register
	__ADS1015_REG_CONFIG_OS_MASK = 0x8000
	__ADS1015_REG_CONFIG_OS_SINGLE = 0x8000 # Write: Set to start a single-conversion
	__ADS1015_REG_CONFIG_OS_BUSY = 0x0000 # Read: Bit = 0 when conversion is in progress
	__ADS1015_REG_CONFIG_OS_NOTBUSY  = 0x8000 # Read: Bit = 1 when device is not performing a conversion

	__ADS1015_REG_CONFIG_MUX_MASK  = 0x7000
	__ADS1015_REG_CONFIG_MUX_DIFF_0_1 = 0x0000 # Differential P = AIN0, N = AIN1 (default)
	__ADS1015_REG_CONFIG_MUX_DIFF_0_3 = 0x1000 # Differential P = AIN0, N = AIN3
	__ADS1015_REG_CONFIG_MUX_DIFF_1_3 = 0x2000 # Differential P = AIN1, N = AIN3
	__ADS1015_REG_CONFIG_MUX_DIFF_2_3 = 0x3000 # Differential P = AIN2, N = AIN3
	__ADS1015_REG_CONFIG_MUX_SINGLE_0 = 0x4000 # Single-ended AIN0
	__ADS1015_REG_CONFIG_MUX_SINGLE_1 = 0x5000 # Single-ended AIN1
	__ADS1015_REG_CONFIG_MUX_SINGLE_2 = 0x6000 # Single-ended AIN2
	__ADS1015_REG_CONFIG_MUX_SINGLE_3 = 0x7000 # Single-ended AIN3

	__ADS1015_REG_CONFIG_PGA_MASK  = 0x0E00
	__ADS1015_REG_CONFIG_PGA_6_144V  = 0x0000 # +/-6.144V range
	__ADS1015_REG_CONFIG_PGA_4_096V  = 0x0200 # +/-4.096V range
	__ADS1015_REG_CONFIG_PGA_2_048V  = 0x0400 # +/-2.048V range (default)
	__ADS1015_REG_CONFIG_PGA_1_024V  = 0x0600 # +/-1.024V range
	__ADS1015_REG_CONFIG_PGA_0_512V  = 0x0800 # +/-0.512V range
	__ADS1015_REG_CONFIG_PGA_0_256V  = 0x0A00 # +/-0.256V range

	__ADS1015_REG_CONFIG_MODE_MASK = 0x0100
	__ADS1015_REG_CONFIG_MODE_CONTIN = 0x0000 # Continuous conversion mode
	__ADS1015_REG_CONFIG_MODE_SINGLE = 0x0100 # Power-down single-shot mode (default)

	__ADS1015_REG_CONFIG_DR_MASK = 0x00E0 
	__ADS1015_REG_CONFIG_DR_128SPS = 0x0000 # 128 samples per second
	__ADS1015_REG_CONFIG_DR_250SPS = 0x0020 # 250 samples per second
	__ADS1015_REG_CONFIG_DR_490SPS = 0x0040 # 490 samples per second
	__ADS1015_REG_CONFIG_DR_920SPS = 0x0060 # 920 samples per second
	__ADS1015_REG_CONFIG_DR_1600SPS  = 0x0080 # 1600 samples per second (default)
	__ADS1015_REG_CONFIG_DR_2400SPS  = 0x00A0 # 2400 samples per second
	__ADS1015_REG_CONFIG_DR_3300SPS  = 0x00C0 # 3300 samples per second (also 0x00E0)

	__ADS1015_REG_CONFIG_CMODE_MASK  = 0x0010
	__ADS1015_REG_CONFIG_CMODE_TRAD  = 0x0000 # Traditional comparator with hysteresis (default)
	__ADS1015_REG_CONFIG_CMODE_WINDOW = 0x0010 # Window comparator

	__ADS1015_REG_CONFIG_CPOL_MASK = 0x0008
	__ADS1015_REG_CONFIG_CPOL_ACTVLOW = 0x0000 # ALERT/RDY pin is low when active (default)
	__ADS1015_REG_CONFIG_CPOL_ACTVHI = 0x0008 # ALERT/RDY pin is high when active

	__ADS1015_REG_CONFIG_CLAT_MASK = 0x0004 # Determines if ALERT/RDY pin latches once asserted
	__ADS1015_REG_CONFIG_CLAT_NONLAT = 0x0000 # Non-latching comparator (default)
	__ADS1015_REG_CONFIG_CLAT_LATCH  = 0x0004 # Latching comparator

	__ADS1015_REG_CONFIG_CQUE_MASK = 0x0003
	__ADS1015_REG_CONFIG_CQUE_1CONV  = 0x0000 # Assert ALERT/RDY after one conversions
	__ADS1015_REG_CONFIG_CQUE_2CONV  = 0x0001 # Assert ALERT/RDY after two conversions
	__ADS1015_REG_CONFIG_CQUE_4CONV  = 0x0002 # Assert ALERT/RDY after four conversions
	__ADS1015_REG_CONFIG_CQUE_NONE = 0x0003 # Disable the comparator and put ALERT/RDY in high state (default)
		
	spsADS1015 = {
	128:__ADS1015_REG_CONFIG_DR_128SPS,
	250:__ADS1015_REG_CONFIG_DR_250SPS,
	490:__ADS1015_REG_CONFIG_DR_490SPS,
	920:__ADS1015_REG_CONFIG_DR_920SPS,
	1600:__ADS1015_REG_CONFIG_DR_1600SPS,
	2400:__ADS1015_REG_CONFIG_DR_2400SPS,
	3300:__ADS1015_REG_CONFIG_DR_3300SPS
	}
	# Dictionariy with the programable gains
	pgaADS1x15 = {
	6144:__ADS1015_REG_CONFIG_PGA_6_144V,
	4096:__ADS1015_REG_CONFIG_PGA_4_096V,
	2048:__ADS1015_REG_CONFIG_PGA_2_048V,
	1024:__ADS1015_REG_CONFIG_PGA_1_024V,
	512:__ADS1015_REG_CONFIG_PGA_0_512V,
	256:__ADS1015_REG_CONFIG_PGA_0_256V
	}	

	# Constructor
	def __init__(self):
		self.i2c = I2C()
	
	def readADCSingleEnded(self, channel = 0):
		pga = 6144
		sps = 250
		
		# With invalid channel return -1
		if (channel > 3):
			return -1
		
		# Disable comparator, Non-latching, Alert/Rdy active low
		# traditional comparator, single-shot mode
		config = self.__ADS1015_REG_CONFIG_CQUE_NONE | \
		 self.__ADS1015_REG_CONFIG_CLAT_NONLAT | \
		 self.__ADS1015_REG_CONFIG_CPOL_ACTVLOW | \
		 self.__ADS1015_REG_CONFIG_CMODE_TRAD  | \
		 self.__ADS1015_REG_CONFIG_MODE_SINGLE 

		config |= self.spsADS1015.setdefault(sps, self.__ADS1015_REG_CONFIG_DR_250SPS)
		config |= self.pgaADS1x15.setdefault(pga, self.__ADS1015_REG_CONFIG_PGA_6_144V)
		self.pga = pga
		
		# Set the channel to be converted
		if channel == 3:
			config |= self.__ADS1015_REG_CONFIG_MUX_SINGLE_3
		elif channel == 2:
			config |= self.__ADS1015_REG_CONFIG_MUX_SINGLE_2
		elif channel == 1:
			config |= self.__ADS1015_REG_CONFIG_MUX_SINGLE_1
		else:
			config |= self.__ADS1015_REG_CONFIG_MUX_SINGLE_0
		
		# Set 'start single-conversion' bit
		config |= self.__ADS1015_REG_CONFIG_OS_SINGLE
		
		# Write config register to the ADC
		bytes = [(config >> 8) & 0xFF, config & 0xFF]
		self.i2c.writeList(self.__ADS1015_REG_POINTER_CONFIG, bytes)
		
		# Wait for the ADC conversion to complete
		# The minimum delay depends on the sps: delay >= 1/sps
		# We add 0.1ms to be sure
		delay = 1.0/sps+0.0001
		time.sleep(delay)
		
		# Read the conversion results
		result = self.i2c.readList(self.__ADS1015_REG_POINTER_CONVERT, 2)
		return ( ((result[0] << 8) | (result[1] & 0xFF)) >> 4 )*pga/2048.0


class I2C:
	def __init__(self):
		self.bus = smbus.SMBus(1)
		
	def writeList(self, reg, list):
		try:
			self.bus.write_i2c_block_data(0x48, reg, list)
		except IOError, err:
			return self.errMsg()

	def readList(self, reg, length):
		try:
			results = self.bus.read_i2c_block_data(0x48, reg, length)
			return results
		except IOError, err:
			return self.errMsg()

