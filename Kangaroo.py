import serial

class Kangaroo:
	def __init__(self):
		#serial port and baudrate is hardcoded for RPi
		try:
			self.serial = serial.Serial("/dev/ttyAMA0")
			self.serial.baudrate = 9600
			self.serial.timeout = 0.5
			self.serial.write("1,start\r\n")
			self.serial.write("2,start\r\n")
			#self.serial.write("1,home\r\n")
		except:
			print("Kangaroo setup failed!")
		
	def units(self, motor, units, lines):
		#accepts motor:1 or 2, units: any integer lines: #encoder lines integer
		command = (str(int(motor)) + "units " + str(int(units)) + " = " + str(int(lines)) + " lines\r\n")
		self.serial.write(command)
	
	def command(self, motor, val):
		#accepts motor:1 or 2, val: any integer  Max range unknown..
		command = (str(int(motor)) + ",s" + str(int(val)) + "\r\n")
		self.serial.write(command)
	
	def speed(self, motor):
		command = (str(int(motor)) + ",gets")
		self.serial.write(command)
		reply = self.serial.readline()
		reply = reply.partition("\r\n")[0]
		reply = reply.partition("s")[2]
		reply = int(reply)
		return reply
		
	def pos(self, motor):
		command = (str(int(motor)) + ",getp")
		self.serial.write(command)
		reply = self.serial.readline()
		reply = reply.partition("\r\n")[0]
		reply = reply.partition("p")[2]
		reply = int(reply)
		return reply
