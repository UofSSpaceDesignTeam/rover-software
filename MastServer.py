from ServoDriver import *
import socket
import time

mastPort = 3004

def parseCommand(command): # parses and executes remote commands
	if command != None:
		if len(command) > 2:
			if command[0] == "#": # is valid
				if command[1] == "M":
					if command[2] == "C": # Camera look
						x_dPad = int(ord(command[3])) - 2	#vertical d-Pad button
						y_dPad = int(ord(command[4])) - 2	#horizontal d-Pad button
					elif command[2] == "S": # Stop
						stopServos()
	else: # command == none
		stopServos()

def stopServos():
	try:
		servoDriver.setServo(3, 1300)
	except:
		raise
		
def stopSockets():
	try:
		mastSocket.close()
	except:
		pass
	try:
		serverSocket.close()
	except:
		pass

def quit():
	stopServos()
	stopSockets()
	exit(0)
		
### Main Program  ###

# set up servo driver
try:
	servoDriver = ServoDriver()
except:
	print("Servo setup failed!")
	quit()
	
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	
try:
	serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	serverSocket.bind(("", mastPort))
	serverSocket.listen(0)
	print("Mast Server listening on port " + str(mastPort))
	while(True):
		(mastSocket, clientAddress) = serverSocket.accept()
		print("Mast Server connected.")
		mastSocket.settimeout(0.5)
		while(True):
			try:
				data = mastSocket.recv(256)
				if(data == ""): # socket closing
					stopServos()
					break
				else:
					parseCommand(data)
			except socket.timeout:
				time.sleep(0.25)
		print("Mast Server disconnected.")
	
except KeyboardInterrupt:
	print("\nmanual shutdown...")
	quit()
except:
	raise
	quit()
