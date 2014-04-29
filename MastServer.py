from ServoDriver import *


servoDriver = ServoDriver()
Pitch = Servo(servoDriver, 0, 100, 2200, 1000)
Yaw = Servo(servoDriver, 1, 1000, 2200, 1596)

while True:
	pos = raw_input('Servo Position?')
	try:
		Pitch.setRelative(int(pos))
	except:
		print("NaN!")


