
#!/usr/bin/python

from ServoDriver import ServoDriver
import socket
import select
import time
import commands

#	define our variables and objects
debug = False

panServo = 14
panMax = 140
panMin = 45
panOffset = 3

tiltServo = 13
tiltMax = 120
tiltMin = 70
tiltOffset = 0

steerServo = 15
steerMax = 115
steerMin = 71
steerOffset = 2

throttleServo = 12
throttleMax = 105
throttleMin = 85
throttleOffset = 5

servo = ServoDriver(0x46)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host = commands.getoutput("/home/pi/python/getIP")
port = 3000

#	make some useful functions

def setup():
	print("Starting script!")
	servo.setPWMFreq(50)
	resetServos()

def resetServos():
	servo.set(panServo, 90 + panOffset)
	servo.set(tiltServo, 90 + tiltOffset)
	servo.set(steerServo, 90 + steerOffset)
	servo.set(throttleServo, 90 + throttleOffset)

def startServer():
	server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	server.bind((host, port))
	server.listen(1)
	print("Server address:\t" + str(host) + ":" + str(port))
	
def getCommand(data):
	if(not '#' in data):
		return None
	command = data.split('#', 1)[1]
	if(len(command) < 7):
		return None
	return command[0:6]
	
def executeCommand(command):
	if("SH" in command):
		resetServos()
		print("Server shutdown by:\t" + str(address[0]))
		client.close()
		server.close()
		quit()
			
	if("RB" in command):
		resetServos()
		print("Server shutdown by:\t" + str(address[0]))
		client.close()
		server.close()
		time.sleep(3)
		print("System rebooting...")
		commands.getoutput("/sbin/shutdown -r now")
		quit()

	if("CM" in command):
		throttle = ord(command[2]) + throttleOffset
		if(throttle > throttleMax):
			throttle = throttleMax
		elif(throttle < throttleMin):
			throttle = throttleMin
		servo.set(throttleServo, throttle)
		
		steering = ord(command[3]) + steerOffset
		if(steering > steerMax):
			steering = steerMax
		elif(steering < steerMin):
			steering = steerMin
		servo.set(steerServo, steering)
		
		pan = ord(command[4]) + panOffset
		if(pan > panMax):
			pan = panMax
		elif(pan < panMin):
			pan = panMin
		servo.set(panServo, pan)
		
		tilt = ord(command[5]) + tiltOffset
		if(tilt > tiltMax):
			tilt = tiltMax
		elif(tilt < tiltMin):
			tilt = tiltMin
		servo.set(tiltServo, tilt)
		return
	
	
#	main execution

setup()
resetServos()
startServer()

try:
	while(True):	# until we quit the server
				
		(client, address) = server.accept()
		deadSocket = False
		data = ""
		command = ""
		print("Connection Established:\t" + str(address[0]))
			
			
		while(True):	# while this connection exists
			
			loopStart = time.time()	# either get a message or time out
			while(time.time() - loopStart < 1):
				dataReady = select.select([client], [], [], 0.1)
				if(dataReady[0]):
					newData = client.recv(100)
					if(newData == ""):
						client.close()
						deadSocket = True
						break
					else:
						data += newData
					
				command = getCommand(data)
				if(command):
					break

			if(deadSocket):
				resetServos()
				print("Connection Closed: " + str(address[0]))
				break
			
			if(command):	# if we have a valid message
				if(debug):
					print("Message Received: " + command)
				data = ""
				client.send("#OK\n")
				executeCommand(command)
			else:
				resetServos()	# shut off motor and center steering
				print("No Message")

except:
	resetServos()
	print("Error! Resetting servos and rebooting...")
	#commands.getoutput("/sbin/shutdown -r now")
	raise
