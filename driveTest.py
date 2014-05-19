from ServoDriver import *
import time

def setSpeed(leftSpeed, rightSpeed):
	if leftSpeed > 127:
		leftSpeed = 127
	if leftSpeed < -127:
		leftSpeed = - 127
	if rightSpeed > 127:
		rightSpeed = 127
	if rightSpeed < -127:
		rightSpeed = -127
	servoDriver,setServo(1,leftSpeed)
	servoDriver,setServo(2, rightSpeed)
	
def stopServos():
	servoDriver.setServo(1, 0)
	servoDriver.setServo(2, 0)
	
def quit():
	stopServos()
	exit(0)
	
try:
	servoDriver = ServoDriver()
except:
	print("Servo setup failed!")
	
try:
	setSpeed(0,127)
	time.sleep(5)
	stopServos()
	time.sleep(1)
	setSpeed(127,127)
	time.sleep(2)
	stopServos()
	time.sleep(1)
	setSpeed(-120,-120)
	time.sleep(5)
	stopServos()

except KeyboardInterrupt:
	print("\nmanual shutdown...")
	quit()
except:
	quit()
	
	