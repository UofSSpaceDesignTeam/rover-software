import serial
import struct
import os

gps = serial.Serial("/dev/ttyAMA0")
gps.baudrate = 9600
gps.timeout = 0.2
fmt = "6s5s1s6s1s5s3s" #time,lat,latdir,lon,londir,alt,prec
encoder = struct.Struct(fmt)

def encodeGPS(data, verbose=0, clear=0):
	try:
		if len(data) > 50:
			if data[4] == "G":
				if clear != 0:
					refreshScreen()
			
				time = data[7:13]
				time = time
				lat = data[18:20] + "." + data[20:22] # + " " + data[22:27] #18-27
				latdir = data[28]
				lon = data [30:33] + "." + data[33:35] #30-40
				londir = data[41]
				quality = data[43]
				numSat = data[45:47]
				prec = data[48:51]
				alt = data[52:57]
				
				packet = encoder.pack(time,lat,latdir,lon,londir,alt,prec)
				
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
		raise
def refreshScreen():
	os.system("clear")

def testResponse(packet):
	if packet is not None:
		print encoder.unpack(packet)

while True:
	position = encodeGPS(gps.readline(),1,1)
	testResponse(position)
