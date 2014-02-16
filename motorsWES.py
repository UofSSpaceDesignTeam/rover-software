import serial
import time

motor = serial.Serial("/dev/ttyAMA0")
motor.baudrate=9600
motor.timeout=3

print motor.name

time.sleep(2)

motor.write("2,start\r\n")
print("start command sent")

motor.write("2,home\r\n")

time.sleep(3)

motor.write("2,getsi\r\n")

print(motor.readline())

while(1):
	time.sleep(10)

	motor.write("2,s1000\r\n")
	print("out")
	#motor.write("2,getp\r\n")
	#print(motor.readline())

	#time.sleep(10)
 
	#motor.write("2,si-1000\r\n")
	#print("in")
	#motor.write("2,getp\r\n")
	#print(motor.readline())

