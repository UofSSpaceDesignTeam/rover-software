import socket
import time
from LSM303 import LSM303

# SET THIS
lat = 12.34
lon = 12.34

lastSendTime = 0.0
course = -999

def getHeading():
	global course
	course = compass.read()
	print(course)
	
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
				print "reading..."
				readGPS()
				print "read"
				sendData()
			except:
				raise	
		print("GPS Server disconnected.")
	
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	quit()
	exit(0)
except:
	quit()
	raise

