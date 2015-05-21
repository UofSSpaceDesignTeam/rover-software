import threading
from Queue import Queue
import serial

class TelemetryThread(threading.Thread):
	def __init__(self, parent):
		threading.Thread.__init__(self)
		self.parent = parent
		self.name = "Telemetry"
		self.mailbox = Queue()
		self.port = serial.Serial("/dev/ttyAMA0", 57600)
		self.messageElements = 18

	def run(self):
		while True:
			data = self.getSerialData()
			if data is not None:
				msg = {"roverGPS":(data["lat"], data["lon"], data["speed"],
					data["heading"])}
				self.parent.commThread.mailbox.put(msg)

	def getSerialData(self):
		inData = ""
		outData = {}
		# wait for message start
		while self.port.read() != "#":
			pass
		# wait for message end
		inChar = self.port.read()
		while inChar != "$":
			inData += inChar
			inChar = self.port.read()
		# parse message
		print inData
		inData = inData.split();
		if len(inData) != self.messageElements:
			return None
		outData["gx"] = float(inData[0])
		outData["gy"] = float(inData[1])
		outData["gz"] = float(inData[2])
		outData["ax"] = float(inData[3])
		outData["ay"] = float(inData[4])
		outData["az"] = float(inData[5])
		outData["mx"] = float(inData[6])
		outData["my"] = float(inData[7])
		outData["mz"] = float(inData[8])
		outData["lat"] = float(inData[9])
		outData["lon"] = float(inData[10])
		outData["speed"] = float(inData[11]) / 60
		outData["alt"] = int(inData[12])
		outData["heading"] = int(inData[13])
		outData["date"] = inData[14]
		outData["time"] = inData[15]
		outData["vout"] = float(inData[16])
		outData["isense"] = float(inData[17])
		outData["laser"] = float(inData[18])
		outData["ph"] = float(inData[19])
		outData["moist"] = float(inData[20])
		return outData

