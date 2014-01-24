#WORK IN PROGRESS!!!
#Proof of Concept to test receiving control data over Sockets from an Xbox Controlled
#Written by Austin Shirley

#TODO: send back data about servo position and speed

import serial
import time
import socket
import RPi.GPIO as GPIO

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

def beginGPIO(): #Edit the GPIO Pin Usage Here
	GPIO.cleanup()
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(7,GPIO.OUT)
	GPIO.setup(11,GPIO.OUT)
	print("GPIO Initialized")

#def writeGPIO(pin,state):
#    if(state == True):
#        GPIO.output(pin,HIGH)
#    else
#        GPIO.output(pin,LOW)

def startSocket(): # begins the server code
	sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	sock.bind((IP,port))
	print("Listening...")
	sock.listen(1)
	# connecting to a client
	conn,addr = sock.accept()
	print("Connected to: ", addr)
	return conn

def parseMotor(data):
	motorCmd=data.partition("A")[2]
	motorCmd=motorCmd.rpartition("B")[0]
	return motorCmd

def parseLED(data):
	LEDCmd=str(data.partition("B")[2])
	return LEDCmd

#Main Program
	
motor=beginSerial()
conn=startSocket()
beginGPIO()
LEDOff=True
	
while(True):
	data=conn.recv(256)
	print("Command Received: ", str(data))
	#Separate the data into its respective strings
	motorCmd=parseMotor(data)
	LEDCmd=parseLED(data)
	print("Motor Command: " + motorCmd + " LED Command: " + LEDCmd)
	if(LEDCmd=="001"):
		GPIO.output(7,GPIO.HIGH)
		print("LED1 ON")
		LEDOff=False
	elif(LEDCmd=="000"):
		GPIO.output(7,GPIO.LOW)
		print("LED1 OFF")
		LEDOff=True
	motor.write(motorCmd) 
	
	#motor.write("1,getp\r\n")
	#print("Servo at Pos: " + motor.readline())

