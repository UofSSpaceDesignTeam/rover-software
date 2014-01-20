# This code is deprecated.  See led.py on Pi2 for more recent implementation of a socket-server

import socket
#import RPi.GPIO as GPIO
#GPIO.setup(7, GPIO.OUT)


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

IP=''
port=5001

sock.bind((IP,port))

while(1):
    data=sock.recvfrom(1024)

    print('data %s')

    datan=int(data)

    if(datan==1):
        #GPIO.output(7,True)
        print("LED ON")
    else:
        #GPIO.output(7,False)
        print("LED OFF")