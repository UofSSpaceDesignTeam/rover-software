from LSM303 import LSM303
from ServoDriver import *

currentYaw = 1600

compass = LSM303(busnum = 0)
servoDriver = ServoDriver()
servoDriver.setServo(4, currentYaw)
print "Antenna bearing is " + str(compass.read())

while True:
	input = raw_input('Enter r to turn right, l to turn left or c to read compass: ')
	if input == "l":
		currentYaw -= 15
		if currentYaw < 800:
			currentYaw = 800
		servoDriver.setServo(4, currentYaw)
	elif input == "r":
		currentYaw += 15
		if currentYaw > 2200:
			currentYaw = 2200
		servoDriver.setServo(4, currentYaw)
	elif input == "c":
		print "Antenna bearing is " + str(round(compass.read()))
		print currentYaw