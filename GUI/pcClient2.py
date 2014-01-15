

# required modules

import socket
import time
import pygame

#       define our variables and objects

throttleDeadzone = 0.02
throttleScale = 15

steeringDeadzone = 0.1
steeringScale = 25

panDeadzone = 0.1
panScale = 40

tiltDeadzone = 0.1
tiltScale = 30

rover = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
roverIP = ("192.168.1.100",3000)
pygame.init()
pygame.joystick.init()
try:
        controller = pygame.joystick.Joystick(0)
except:
        print("XBox controller not found!")
        time.sleep(3)
        quit()
print("Using: " + controller.get_name())

#       make some useful functions

def connect():
        print("Connecting to " + roverIP[0] + ":" + str(roverIP[1]))
        while(True):
                try:
                        rover.connect(roverIP)
                        break
                except socket.error:
                        print("Retrying...")
                        time.sleep(3)

        print("Connection Established")

def getinput():
        pygame.event.pump()
        
        throttle = controller.get_axis(2)
        if(throttle < throttleDeadzone and throttle > -throttleDeadzone):
                throttle = 0
        elif(throttle > throttleDeadzone):
                throttle -= throttleDeadzone
        else:
                throttle += throttleDeadzone         
        throttle *= throttleScale
        throttle = int(throttle) + 90
        if(throttle > 255):
                throttle = 255
        if(throttle < 0):
                throttle = 0
                
        steering = controller.get_axis(0)
        if(steering < steeringDeadzone and steering > -steeringDeadzone):
                steering = 0
        elif(steering > steeringDeadzone):
                steering -= steeringDeadzone
        else:
                steering += steeringDeadzone       
        steering *= steeringScale
        steering = int(steering) + 90
        if(steering > 255):
                steering = 255
        if(steering < 0):
                steering = 0

        pan = controller.get_axis(4)
        if(pan < panDeadzone and pan > -panDeadzone):
                pan = 0
        elif(pan > panDeadzone):
                pan -= panDeadzone
        else:
                pan += panDeadzone       
        pan *= panScale
        pan = int(pan) + 90
        if(pan > 255):
                pan = 255
        if(pan < 0):
                pan = 0

        tilt = -1.0 * controller.get_axis(3)
        if(tilt < tiltDeadzone and tilt > -tiltDeadzone):
                tilt = 0
        elif(tilt > tiltDeadzone):
                tilt -= tiltDeadzone
        else:
                tilt += tiltDeadzone       
        tilt *= tiltScale
        tilt = int(tilt) + 90
        if(tilt > 255):
                tilt = 255
        if(tilt < 0):
                tilt = 0
                
        return(throttle, steering, pan, tilt)


#       main execution

controller.init()
connect()

try:
        while(True):
                (throttle, steering, pan, tilt) = getinput()
                rover.send("#CM" + chr(throttle) + chr(steering) + chr(pan) + chr(tilt))
                time.sleep(0.05)

except:
        print("closing connection")
        rover.shutdown(socket.SHUT_RDWR)
        rover.close()
        time.sleep(3)
        raise

