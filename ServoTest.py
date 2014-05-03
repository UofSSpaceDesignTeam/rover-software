from ServoDriver import *
import time

servoDriver = ServoDriver()

## Test Code
while True:
	pos = raw_input('Pulse Length Position?')
	try:
		servoDriver.setServo(8, int(pos))
	except:
		print("NaN!")
