import threading
import json
from Queue import Queue
import time
import smbus
import roverMessages

# matching structures from arduino
class CommandType:
	stop = 0x00
	setPos = 0x01
	setAbs = 0x02
	
class Command:
	def __init__(self):
		self.header = 0xF7
		self.type = 0x00
		self.d1 = 0x0000
		self.d2 = 0x0000
		self.d3 = 0x0000
		self.d4 = 0x0000
		self.csum = 0x00
		self.trailer = 0xF8

class ArmThread(threading.Thread):
	def __init__(self, parent, i2cSemaphore):
		threading.Thread.__init__(self)
		self.parent = parent
		self.period = 0.05
		self.mailbox = Queue()
		self.i2cSem = i2cSemaphore
		self.i2c = smbus.SMBus(1)
		self.i2cAddress = 0x08

	def run(self):
		command = Command()
		baseSpeed = None
		dx = None
		dy = None
		dz = None
		dphi = None
		x = None
		y = None
		z = None
		phi = None
		absMode = False
		while True:
			time.sleep(self.period)
			while not self.mailbox.empty():
				data = self.mailbox.get()
				#print data
				if "c2j1x" in data:
					dx = int(data["c2j1x"] * 10) # -255 to 255
					#print baseSpeed
				elif "c2j1y" in data:
					dy = int(data["c2j1y"] * 10)
					#print L1
				elif "c2j2x" in data:
					dz = int(data["c2j2x"] * 10)
					#print L2
				elif "c2j2y" in data:
					dphi = int(data["c2j2y"] * 10)
				elif "arm-gui_x" in data:
					x = int(data["arm-gui_x"])
				elif "arm-gui_y" in data:
					y = int(data["arm-gui_y"])
				elif "arm-gui_z" in data:
					z = int(data["arm-gui_z"])
				elif "arm-gui_phi" in data:
					phi = int(data["arm-gui_phi"])
				elif "AbsEnable" in data:
					absMode = data["AbsEnable"]
			
			if not absMode and dx is not None and dy is not None and dz is not None and dphi is not None:
				command.type = CommandType.setPos
				command.d1 = int(dx)
				command.d2 = int(dy)
				command.d3 = int(dz)
				command.d4 = int(dphi)
				dx = None
				dy = None
				dz = None
				dphi = None
				self.sendCommand(command)
			
			elif absMode and x is not Node and y is not None and z is not Node and phi is not None:
				command.type = CommandType.setAbs
				command.d1 = int(x)
				command.d2 = int(y)
				command.d3 = int(z)
				command.d4 = int(phi)
				x = None
				y = None
				z = None
				phi = None
				self.sendCommand(command)

	def sendCommand(self, command):
		command.csum = (command.type + command.d1 + command.d2 + command.d3 + command.d4) % 256
		try:
			self.i2cSem.acquire()
			self.i2c.write_byte(self.i2cAddress, command.header)
			self.i2c.write_byte(self.i2cAddress, command.type)
			self.i2c.write_byte(self.i2cAddress, command.d1 & 0xFF)
			self.i2c.write_byte(self.i2cAddress, command.d1 >> 8)
			self.i2c.write_byte(self.i2cAddress, command.d2 & 0xFF)
			self.i2c.write_byte(self.i2cAddress, command.d2 >> 8)
			self.i2c.write_byte(self.i2cAddress, command.d3 & 0xFF)
			self.i2c.write_byte(self.i2cAddress, command.d3 >> 8)
			self.i2c.write_byte(self.i2cAddress, command.d4 & 0xFF)
			self.i2c.write_byte(self.i2cAddress, command.d4 >> 8)
			self.i2c.write_byte(self.i2cAddress, command.csum)
			self.i2c.write_byte(self.i2cAddress, command.trailer)	
		except IOError:
			print("Arm thread got IOError")
		self.i2cSem.release()

