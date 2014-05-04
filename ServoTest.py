from ServoDriver import *
import time

servoDriver = ServoDriver()

## Test Code
while True:
	pos1 = raw_input('Pulse Length Position?')
	pos2 = raw_input('Pulse Length Position?')
	pos3 = raw_input('Pulse Length Position?')
	try:
		servoDriver.setServo(8,int(pos1))
		servoDriver.setServo(9,int(pos2))
		servoDriver.setServo(10,int(pos3))
	except:
		print("NaN!")
