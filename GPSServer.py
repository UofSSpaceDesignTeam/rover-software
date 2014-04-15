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

def readGPS():
	global gps, latitude, longitude
	data = gps.read(gps.inWaiting())
	dataStart = data.find("GGA")
	if dataStart != -1:	# found start of valid sentence
		dataEnd = data.find("*", dataStart)
		if dataEnd != -1 and dataEnd - dataStart < 70:
			data = data[dataStart:dataEnd]
			print(data)
			values = data.split(",")
			latitude = float(values[2][0:1]) + float(values[2][2:7]) / 60.0
			if values[3] == "S":
				latitude = -latitude
			longitude = float(values[4][0:1]) + float(values[4][2:7]) / 60.0
			if values[5] == "E":
				longitude = -longitude

def sendData():
	global latitude, longitude, serverSocket, logfile
	if latitude != None and longitude != None:
		serverSocket.send(struct.pack("!ff", latitude, longitude)) 
		try:
			global logfile
			logfile.write(str(latitude) + "," + str(longitude) + "\n")
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
	logfile = file("gpsLog" + time.strftime("%m%d%H%M%S", time.localtime()),"w")
	time.sleep(0.2)
	logfile.open()
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
