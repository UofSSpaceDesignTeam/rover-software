# Proof of concept to toggle GPIO pins on the PI using the rover network.
# The client connects to a server running on the Pi and sends simple strings using Sockets
# Written by Austin

#To Do: Handle client disconnects more effectively (often crashes the server).

import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #activates UDP mode for the socket
#Connects to Pi2
IP='192.168.1.104'
#Not official port
port=5001
#onMessage="on"
#offMessage="off"

#Start client Socket
sock.connect((IP,port))

while(1):
    try:
        kb=input("Press 1, 2, 3 or 4 to control LEDs")
        kb=str(kb)
		#Send the whole string in however may packets are needed.  Must be a string.
        sock.sendall(kb)
    except:
        print("something went wrong...")