import serial
import socket
import struct
import os
import sys

fmt = "6s8s9s5s3s" #time,lat,latdir,lon,londir,alt,prec
commandPort = 3004
GPSPort = 3005
serialPort = "/dev/ttyAMA0" #/dev/ttyAMA0
GPSIP = "127.0.0.1"
header = "#GD" #GPS Data

def encodeGPS(data, verbose=0, clear=0):
	try:
		if len(data) > 50:
			if data[4] == "G":
				if clear != 0:
					refreshScreen()
			
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
				
				if verbose != 0:
					print "Time taken at " + time + " UTC"
					print "Lattidude:    " + lat,latdir
					print "Longitude:    " + lon,londir
					print "Status:       " + quality
					print numSat + " Sattelites Connected"
					print "Precision:    " + prec + "m"
					print "Altitude:     " + alt + "m"
					print "\r"
					print "Encoded Data: " + packet + "\r\n"


				return packet
				
	except:
		print("Invalid Response")
		return None
		
def parseCommand(command):
	if(len(command) > 2 and command[0] == "#"): # is valid
		if(command[1] == "G"):
			if(command[2] == "S"): #GS
				print("starting GPS data")
				startGPS()
			elif(command[2] == "E"): #GE
				print("stopping GPS data")
				stopGPS()
			elif(command[2]== "P"): #GL
				print("logging GPS data")
				logGPS()

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
	
## Test Functions
def refreshScreen():
	os.system("clear")

def testResponse(packet):
	if packet is not None:
		print encoder.unpack(packet)
## End Test		
		
### Main Program ###
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
logfile = file("gps.log","w+b")
encoder = struct.Struct(fmt)

try:
	gps = serial.Serial(serialPort, bytesize = 8, parity = 'N', stopbits = 1)
	gps.baudrate = 9600
	gps.timeout = 0.2
	
except:
	print("GPS serial setup failed!")
	raise
	#subprocess.call("sudo reboot", shell = True)

try:
	serverSocket.bind(("", commandPort))
	serverSocket.listen(0)
	print("GPSServer port: " + str(commandPort))
	print("GPSData port: " + str(GPSPort))
	while(True):
		(commandSocket, clientAddress) = serverSocket.accept()
		print("Connected to: " + str(clientAddress[0]))
		while(True):
			data = commandSocket.recv(256)
			if(data == ""): # socket closing
				stopGPS()
				break
			else:
				parseCommand(data)
				sendData()
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
