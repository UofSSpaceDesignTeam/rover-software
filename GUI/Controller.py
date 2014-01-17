#!/usr/bin/python

  # dependency list

import pygame

  # global variables

leftJoystickDeadzoneX = 0.1
leftJoystickDeadzoneY = 0.1
rightJoystickDeadzoneX = 0.1
rightJoystickDeadzoneY = 0.1
triggerDeadzone = 0.1



  # function definitions

def setup():	# connect to xbox controller if one exists
	pygame.init()
	pygame.joystick.init()
	try:
	    controller = pygame.joystick.Joystick(0)
	except:
	  

def getAxes():	# returns a tuple of all joystick / trigger data (-1.0 to 1.0)

def getTriggers():
	# return tuple of 
def getButtons():
	if(not '#' in data):
		return None
	command = data.split('#', 1)[1]
	if(len(command) < 7):
		return None
	return (command[0:6],5)
	
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
