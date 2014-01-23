import serial
import time
import socket

# variables
IP = ''
port = 5001
baudrate = 9600
timeout = 3


def beginSerial():
	motor = serial.Serial("/dev/ttyAMA0")
	motor.baudrate = baudrate
	motor.timeout = timeout
	print(motor.name)
	motor.write("1,start\r\n")
	print("Start command sent!")
	return motor
	
	#motor.write("1,getsi\r\n")
	#print(motor.readline())

def startSocket(): # begins the server code
	sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	sock.bind((IP,port))
	print("Listening...")
	sock.listen(1)
	# connecting to a client
	conn,addr = sock.accept()
	print("Connected to: ", addr)
	return conn
	
# main program

motor=beginSerial()
conn=startSocket()
	
while(True):
	data=conn.recv(256)
	print("Command Recieved: ", str(data))
	motor.write(data)
	#motor.write("1,getp\r\n")
	#print("Servo at Pos: " + motor.readline())

