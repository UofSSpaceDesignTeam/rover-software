import subprocess
import socket
from threading import Thread

server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
cam = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
motorPort = 3000
camPort = 3001
IP = ''

def connectListener():
	global com
	global addr
	print("listening for connection")
	server.listen(1)
	com,addr = server.accept()
	print("got connection")

def connectCamera():
	global p
	command = "raspivid -b 500000 -ex fixedfps -fps 20 -t 0 -rot 180 -o - | nc 192.168.1.4 3001"
	p = subprocess.Popen(str(command),shell=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)

def parseCommand(datan)
	if(datan[1] == "C"):#camera/control stuff
		if(datan[2] == "S"):
			connectCamera()
			print("Camera connected")
		elif(datan[2] == "C"):
			print("Camera disconnecting")
			p.stdout.flush
			p.stdout.close
			p.kill()
			p.terminate()
#	elif(datan[1] == "C"):#control stuff

server.bind((IP,motorPort))
connectListener()
while True:
	data = com.recv(1024)
	print("Data Recieved ",str(data))
	try:
		datan = str(data)
		datan = datan.split(" ")
		parseCommand(datan)
	except:
		print("invalid input")

