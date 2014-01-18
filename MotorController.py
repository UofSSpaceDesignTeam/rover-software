import serial
import time

motor = serial.Serial("/dev/ttyAMA0")
motor.baudrate=9600
motor.timeout=3

print motor.name

time.sleep(2)

motor.write("1.start\r\n")
print("start command sent")

motor.write("1,home\r\n")

time.sleep(3)

motor.write("1,getsi\r\n")

print(morot.readline())

time.sleep(2)

while(1):
	motor.write("1,s500\r\n")
	for i in range(0,20):
		motor.write("1,getp\r\n")
		print(motor.readline())
		motor.write("1,gets\r\n")
		print(motor.readline())
		time.sleep(.5)

	motor.write("1,s-500\r\n")
	for j in range(0,20):
		motor.write("1,getp\r\n")
		print(motor.readline())
		motor.write("1,gets\r\n")
		print(motor.readline())
		time.sleep(.5)