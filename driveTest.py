from ServoDriver import *
import time

def setSpeed(leftSpeed, rightSpeed):
	servoDriver.setServo(4,leftSpeed)
	servoDriver.setServo(5, rightSpeed)
	
def stopServos():
	servoDriver.setServo(4, 1570)
	servoDriver.setServo(5, 1570)
	
def quit():
	stopServos()
	exit(0)
	
global servoDriver
try:
	servoDriver = ServoDriver()
except:
	print("Servo setup failed!")
	raise
	
try:
	setSpeed(1950,1400)
	time.sleep(5)
	stopServos()

except KeyboardInterrupt:
	print("\nmanual shutdown...")
	quit()
except:
	raise
	quit()
	
