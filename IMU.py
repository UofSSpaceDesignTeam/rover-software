import smbus

class IMU:
	def __init__(self):
		self.power_mgmt_1 = 0x6b
		self.power_mgmt_2 = 0x6c
		self.address = 0x68
		self.scale = 16384.0
		self.bus = smbus.SMBus(1)
		self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)
	
	def read_word(self, adr):
		high = self.bus.read_byte_data(self.address, adr)
		low = self.bus.read_byte_data(self.address, adr+1)
		val = (high << 8) + low
		return val
	
	def read_word_2c(self, adr):
		val = self.read_word(adr)
		if (val >= 0x8000):
			return -((65535 - val) + 1)
		else:
			return val
	
	def dist(self, a, b):
		return (a ** 2 + b ** 2) ** 0.5
	
	def get_y_rotation(self,x,y,z):
		radians = math.atan2(x, self.dist(y, z))
		return -math.degrees(radians)
	
	def pitch(self):
		self.accel_xout_scaled = self.read_word_2c(0x3b)/self.scale
		self.accel_yout_scaled = self.read_word_2c(0x3d)/self.scale
		self.accel_zout_scaled = self.read_word_2c(0x3f)/self.scale
		return self.get_y_rotation(self.accel_xout_scaled, self.accel_yout_scaled, self.accel_zout_scaled)

