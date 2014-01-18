import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #activates UDP mode for the socket

IP='192.168.1.104'
port=5001
onMessage="on"
offMessage="off"


sock.connect((IP,port))

while(1):
    try:
        kb=input('1 for on, 0 for off')
        kb=str(kb)
        sock.sendall(kb)
    except:
        print("something went wrong...")