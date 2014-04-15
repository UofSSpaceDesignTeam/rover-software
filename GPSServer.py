# Sends location information periodically to connected clients.

import serial
import socket
import time
import struct

GPSPort = 3005
header = "#GD" #GPS Data
lastSendTime = 0.0
gps = None
logfile = None

latitude = None
longitude = None
altitude = None
cep = None

def readGPS():
	global gps, latitude, longitude, altitude, cep
	rawData = gps.read(gps.inWaiting())
	#print(rawData)
	dataStart = rawData.find("GGA")
	if dataStart != -1:	# found start of valid sentence
		dataEnd = min(dataStart + 70, len(rawData) - dataStart - 2)
		data = rawData[dataStart:dataEnd]
		#print(data)
		values = data.split(",")
		if len(values) > 9:
			latitude = float(values[2][0:2]) + float(values[2][2:7]) / 60.0
			if values[3] == "S":
				latitude *= -1.0
			longitude = float(values[4][0:3]) + float(values[4][2:7]) / 60.0
			if values[5] == "W":
				longitude *= -1.0
			cep = float(values[8]) * 2.5 # 50% confidence error circle
			altitude = float(values[9])
			#print(latitude)
			#print(longitude)
			#print(altitude)
			#print(cep)

def sendData():
	global latitude, longitude, altitude, cep, dataSocket, logfile
	if latitude != None and longitude != None and altitude != None and cep != None:
		dataSocket.send(struct.pack("!ffff", latitude, longitude, altitude, cep))
		try:
			global logfile
			logfile.write(str(latitude) + "," + str(longitude) + "," + str(altitude) + "\n")
		except:
			pass
			
def stopSockets():
	try:
		commandSocket.close()
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

### Main Program ###

# set up logging
try:
	logfile = open("/home/pi/gpsLogs/" + time.strftime("%m%d%H%M%S", time.localtime()) + ".log", "w")
except:
	print("GPS logging failed.")

# set up serial connection
try:
	gps = serial.Serial("/dev/ttyAMA0", bytesize = 8, parity = 'N', stopbits = 1)
	gps.baudrate = 9600
	gps.timeout = 0.2
except:
	print("GPS setup failed!")
	raise

# start socket
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
	serverSocket.bind(("", GPSPort))
	serverSocket.listen(0)
	print("GPS Server listening on port " + str(GPSPort))
	while(True):
		(dataSocket, clientAddress) = serverSocket.accept()
		print("GPS Server connected.")
		while(True):
			time.sleep(2.0)
			readGPS()
			try:
				sendData()
			except:
				break	
		print("GPS Server disconnected.")
	
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	stopGPS()
	stopLog()
	stopSockets()
except socket.error as e:
	print(e.strerror)
	stopGPS()
	stopLog()
	stopSockets()
except:
	stopGPS()
	stopLog()
	stopSockets()
	raise
