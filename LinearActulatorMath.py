# math to calculate the linear actuator speed and length
# to be implimeted in the main program

import math

# constants, in mm 
A1 = 372.0
B1 = 105.0
A2 = 121.0
B2 = 363.0


# values for variables have to be added
th1 = 0.1           # max = .1
L1 = 400.0          # has to be between 283.5  and 446.5 mm
th2 = 0.03          # max = .03
L2 = 400.0          # has to between 380.8 and 545.9  mm 

# L1p,L2p is speed of the the actuator 
# L1,L2 is position of actuator 
# th1,1h2 (theta) is the angular speed 


# calculates the speed the actuator needs to move to get the wanted angular speed 
# the max actuator speed is approx. 10.16 mm/sec

L1p = (th1*A1*B1)/(L1) * math.sqrt( (1 - pow( ( (pow(A1,2)+pow(B1,2)-pow(L1,2))/(2*A1*B1)),2)))
if L1p >= 10:
    L1p = 10 
print L1p
L2p  = (th2*A1*B2)/(L2) * math.sqrt( (1 - pow( ((pow(A2,2)+pow(B2,2)-pow(L2,2))/(2*A2*B2)),2)))#if L2p >= 10:
if L2p >= 10:
    L2p = 10
print L2p 

