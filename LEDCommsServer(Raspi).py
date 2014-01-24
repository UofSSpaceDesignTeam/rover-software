#Simple example server for control of GPIO Pins using Sockets
#ToDo: make the server able to close a connection if the host disconnects spontaneously and listen again.
#By Austin Shirley

import RPi.GPIO as GPIO
import socket

sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

#sets up GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)

print("begin")


IP=''
port=5001

#configure the socket as a server to listen for (1) connection from a client
sock.bind((IP,port))
sock.listen(1)

#copies the socket to a connected instance for data collection 
conn,addr = sock.accept()
print("Connected to ", addr)

#cases for turning on and off LEDs using the new conn socket
while(1):
	data=conn.recv(1024)
	print("Data Recieved ",str(data))
	
	try:
		datan=int(data)
	except:
		print("invalid input")
	
	if(datan==1):
		GPIO.output(7,GPIO.HIGH)
		print("LED1 ON")
	if(datan==2):
		print("LED2 ON")
		GPIO.output(11,GPIO.HIGH)
	if(datan==0):
		GPIO.output(7,GPIO.LOW)
		print("LED1 OFF")
	if(datan==3):
		GPIO.output(11,GPIO.LOW)
		print("LED2 OFF")
	if not data: break

conn.close()

#turning off LEDs
GPIO.output(7,GPIO.LOW)
GPIO.output(11,GPIO.LOW)
