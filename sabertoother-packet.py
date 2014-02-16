# Packet-Based Serial protocol for Sabertooth Motor Controller

import serial
import time

drive=serial.Serial("/dev/ttyAMA0")

speedR = 0
speedL = 0
speed = 0

# Direction Commands (Right Forwards, Left Forwards etc)
RF = 0
RR = 1
LF = 4
LR = 5

# Motor Controller Adresses (m1: front wheels, m2 middle wheels etc)
m1 = 128
m2 = 129
m3 = 130

def sabertoothSend(address, command, speed):
	checksum=int(address)+int(command)+int(speed) & 127
	drive.write(chr(int(address)))
	drive.write(chr(int(command)))
	drive.write(chr(int(speed)))
	drive.write(chr(int(checksum)))
def sabertoothKill():
	# Disclamer, no extinct Tigers were harmed in the making of this function
	address=127
	while(address <= 135):
		sabertoothSend(address,RF,0)
		sabertoothSend(address,LF,0)
		address = address + 1
	print("All Motors Stopped (Hopefully)")


try:
	print("start")
	while(1):
	
		while(speed < 50):
			sabertoothSend(m1,RF,speed)
			sabertoothSend(m1,LF,speed)
			speed = speed + 2
			time.sleep(.1)
			print(speed)
		while(speed > 1):
			sabertoothSend(m1,RF,speed)
			sabertoothSend(m2,LF,speed)
			speed = speed - 2
			time.sleep(.1)
			print(speed)

except:
	sabertoothKill()
