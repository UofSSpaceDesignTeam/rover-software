# Simplified Serial Protocol for Sabertooth Motor Controller

#import RPi.GPIO as GPIO
import serial
import time

drive=serial.Serial("/dev/ttyAMA0")

speed = 64

def sp(speed):
	drive.write(chr(int(speed)))
	delay(1)


try:
	print("start")
	while(1):
	
		while(speed < 127):
			drive.write(chr(int(speed)))
			speed = speed + 1
			time.sleep(.1)
			print(speed)
		while(speed > 1):
			drive.write(chr(int(speed)))
			speed = speed - 1
			time.sleep(.1)
			print(speed)

except:
	drive.write(chr(int(0)))
