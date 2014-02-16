# Small utility to send some fake data over serial
# Makes random sinusoidal-ish output
# Adapted by Austin Shirley

import serial
import random, time, math


port = "/dev/ttyAMA0"
ser = serial.Serial(port, 57600)

incycle = 0

while True:
    t = int(random.randint(60, 80) * (1 + math.sin(incycle)))
    x = ser.write(chr(t))
    time.sleep(0.02)
    
    incycle += 0.01
    if incycle >= 2 * math.pi:
        incycle = 0


ser.close()
