from ServoDriver import *

try:
	ServoDriver = ServoDriver()
	Pitch = Servo(servoDriver, 0, 1000, 2200, 1596)
	Yaw = Servo(servoDriver, 1, 1000, 2200, 1596)
except:
	print("Servo setup failed")

while True:
	Pitch.setRelative(2)


