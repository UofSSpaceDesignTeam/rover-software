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
	dataStart = rawData.find("GGA")
	if dataStart != -1:	# found start of valid sentence
		dataEnd = rawData.find("*", dataStart)
		if dataEnd != -1 and dataEnd - dataStart < 70:
			data = rawData[dataStart:dataEnd]
			print(data)
			values = data.split(",")
			latitude = float(values[2][0:1]) + float(values[2][2:7]) / 60.0
			if values[3] == "S":
				latitude *= -1.0
			longitude = float(values[4][0:1]) + float(values[4][2:7]) / 60.0
			if values[5] == "E":
				longitude *= -1.0
			cep = float(values[8]) * 2.5 # 50% confidence error circle
			altitude = float(values[9])

def sendData():
	global latitude, longitude, altitude, cep, serverSocket, logfile
	if latitude != None and longitude != None:
		serverSocket.send(struct.pack("!ffff", latitude, longitude, altitude, cep)) 
		try:
			global logfile
			logfile.write(str(latitude) + "," + str(longitude) + "," + str(altitude) "\n")
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
	time.sleep(0.2)
except:
	print("Could not set up log file.")

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
	print("using serial port " + gps.name)
	print("DriveServer listening on port " + str(GPSPort))
	while(True):
		(dataSocket, clientAddress) = serverSocket.accept()
		print("Connected to: " + str(clientAddress[0]))
		while(True):
			time.sleep(2.0)
			readGPS()
			try:
				sendData()
			except:
				raise
				break	
		print("Connection to: " + str(clientAddress[0]) + " closed")
		#encode = encodeGPS(gps.readline(),1,1)
		#decode = testResponse(encode)
		#if decode is not None:
		#	print decode

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
