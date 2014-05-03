from ServoDriver import *
import time

## Test Code
while True:
	pos = raw_input('Pulse Length Position?')
	try:
		servoDriver.setServo(3, int(pos))
	except:
		print("NaN!")