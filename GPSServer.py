# Sends location information periodically to connected clients.

import serial
import socket
import struct
import time

fmt = "6s8s9s5s3s" #time,lat,lon,alt,prec
GPSPort = 3005
header = "#GD" #GPS Data

def encodeGPS(data, verbose=0, clear=0):
	try:
		if len(data) > 50:
			if data[4] == "G":			
				time = data[7:13]
				time = time
				lat = str(round(float(data[18:20]) + float(data[20:27])/60,5)) #18-27
				latdir = data[28]
				lon = str(round(float(data[30:33]) + float(data[33:40])/60,5)) #30-40
				londir = data[41]
				quality = data[43]
				numSat = data[45:47]
				prec = data[48:51]
				alt = data[52:57]
				packet = encoder.pack(time,lat,lon,alt,prec)
				if verbose:
					print "Time taken at " + time + " UTC"
					print "Latitude:    " + lat,latdir
					print "Longitude:    " + lon,londir
					print "Status:       " + quality
					print numSat + " Satellites Connected"
					print "Precision:    " + prec + "m"
					print "Altitude:     " + alt + "m"
					print "\r"
					print "Encoded Data: " + packet + "\r\n"
				return packet	
	except:
		print("Invalid GPS data")
	return None

def sendData():
	position = encodeGPS(gps.readline())	
	if position is not None:
		if logfile.closed() == False:
			try:
				new_data = position
				logfile.write(new_data[2])
			except:
				print "Could not write to file!"
		try:
			serverSocket.send(header + position)
			return True
		except:
			stopGPS()
			
def stopSockets():
	try:
		commandSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass
		
def startGPS():
	gps.open()
	rawSerial = gps.readline()

def stopGPS():
	gps.close()
	
def logGPS(data): #Saves GPS data to a local file
	try:
		logfile.open()
	except:
		print("Could not create gps.log file!")

def testResponse(packet):
	if packet is not None:
		print encoder.unpack(packet)
		
		
### Main Program ###

logfile = file("gpsLog" + time.strftime("%m%d%H%M%S", time.localtime()),"w+b")
time.sleep(0.2)
try:
	logfile.open()
except:
	print("Could not open log file.")

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
	gps = serial.Serial("/dev/ttyAMA0", bytesize = 8, parity = 'N', stopbits = 1)
	gps.baudrate = 9600
	gps.timeout = 0.2
	encoder = struct.Struct(fmt)
except:
	print("GPS setup failed!")
	raise
	#subprocess.call("sudo reboot", shell = True)

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
			try:
				sendData()
			except:
				break	
		print("Connection to: " + str(clientAddress[0]) + " closed")
		#encode = encodeGPS(gps.readline(),1,1)
		#decode = testResponse(encode)
		#if decode is not None:
		#	print decode

except KeyboardInterrupt:
	print("\nmanual shutdown...")
	stopGPS()
	stopSockets()
except socket.error as e:
	print(e.strerror)
	stopGPS()
	stopSockets()
	#subprocess.call("sudo reboot", shell = True)
except:
	stopGPS()
	stopSockets()
	#subprocess.call("sudo reboot", shell = True)
	raise
