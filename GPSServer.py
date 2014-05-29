import serial
import socket
import time
import struct
from LSM303 import LSM303

GPSPort = 3005
header = "#GD" #GPS Data
lastSendTime = 0.0
gps = None
logfile = None

latitude = None
longitude = None
altitude = None
hdop = None
course = -999

def readGPS():
	global gps, latitude, longitude, altitude, hdop, course
	try:
		course = compass.read()
	except:
		pass
	rawData = gps.read(gps.inWaiting())
	print str(rawData)
	dataStart = rawData.find("GGA")
	if dataStart != -1:	# found start of valid sentence
		dataEnd = min(dataStart + 70, len(rawData) - dataStart - 2)
		data = rawData[dataStart:dataEnd]
		values = data.split(",")
		if len(values) > 9:
			latitude = float(values[2][:2]) + float(values[2][2:]) / 60.0
			if values[3] == "S":
				latitude *= -1.0
			longitude = float(values[4][:3]) + float(values[4][3:]) / 60.0
			if values[5] == "W":
				longitude *= -1.0
			hdop = float(values[8])
			altitude = float(values[9])

def sendData():
	global latitude, longitude, altitude, hdop, course, dataSocket, logfile
	if latitude != None and longitude != None and altitude != None and hdop != None:
		dataSocket.send(struct.pack("!fffff", latitude, longitude, altitude, hdop, course))
		try:
			logfile.write(str(latitude) + "," + str(longitude) + "," + str(altitude) + "," + str(hdop) + "\n")
		except:
			pass
	
def stopSockets():
	try:
		dataSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass

def stopGPS():
	try:
		gps.close()
	except:
		pass

def stopLog():
	try:
		logfile.close()
	except:
		pass

def quit():
	stopGPS()
	stopLog()
	stopSockets()

### Main Program ###

# set up logging
try:
	logfile = open("/home/pi/gpsLogs/" + time.strftime("%m%d%H%M%S", time.localtime()) + ".log", "w+")
except:
	print("GPS logging failed.")
	pass

# set up serial connection
try:
	gps = serial.Serial("/dev/ttyAMA0", bytesize = 8, parity = 'N', stopbits = 1)
	gps.baudrate = 9600
	gps.timeout = 0.2
except:
	print("GPS setup failed!")
	quit()

# set up compass
try:
	compass = LSM303()
except:
	print("compass setup failed!")

# start socket
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
	serverSocket.bind(("", GPSPort))
	serverSocket.listen(0)
	print("GPS Server listening on port " + str(GPSPort))
	while(True):
		(dataSocket, clientAddress) = serverSocket.accept()
		print("GPS Server connected.")
		while(True):
			time.sleep(2.0)
			try:
				readGPS()
				sendData()
			except:
				break	
		print("GPS Server disconnected.")
	
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	quit()
	exit(0)
except:
	quit()
	raise

