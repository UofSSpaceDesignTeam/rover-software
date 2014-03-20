# math to calculate the linear actuator speed and length
# to be implemented in the main program

import math

# constants, in mm 
A1 = 372.0
B1 = 105.0
A2 = 121.0
B2 = 363.0


# values for variables have to be added
th1_dot = 0.1       # max = + and - .1, to be imputed from control  
L1 = 446.5               # has to be between 283.5  and 446.5 mm
th2_dot = 0.03     # max = + and - .03, to be imputed from control 
L2 = 484         # has to between 380.8 and 445.9  mm 

# L1p,L2p is speed of the the actuator 
# L1,L2 is position of actuator, this will change as the position moves
# th1_dot,th2_dot (theta) is the angular speed 


# calculates the speed the actuator needs to move to get the wanted angular speed 
# the max actuator speed is approx. 10.16 mm/sec

L1p = (th1_dot*A1*B1)/(L1) * math.sqrt( (1 - pow( ( (pow(A1,2)+pow(B1,2)-pow(L1,2))/(2*A1*B1)),2)))
if L1p >= 10:
    L1p = 10
elif L1p <= -10:
    L1p = -10
print L1p
L2p  = (th2_dot*A1*B2)/(L2) * math.sqrt( (1 - pow( ((pow(A2,2)+pow(B2,2)-pow(L2,2))/(2*A2*B2)),2)))
if L2p >= 10:
    L2p = 10
elif L2p <= -10:
    L2p = -10 
print L2p 


# calculates the ratio of angular speed to move the end point forward and back 
# th1 and th2 are the angles of the arm, the angles are constantly changing 
# c1 and c2 are constants 
# should set th2_dot to a constant speed
c1 = 100	# value needs to be found 
c2 = 100	# value needs to be found 
th1 = 75
th2 = 80

th1_dot = th2_dot * (- ( c1*math.sin(th1 + th2) ) / (c1*math.sin(th1) + c2*math.sin(th1 + th2)) )
if th1_dot >= .1:
    th1_dot = .1
elif th1_dot <= -.1:
    th1_dot = -.1
print th1_dot


# the ratio to move the end point up and down 

th1_dot = th2_dot * (- ( c1*math.cos(th1 + th2) ) / (c1*math.cos(th1) + c2 * math.sin(th1 + th2)) )
if th1_dot >= .1:
    th1_dot = .1
elif th1_dot <= -.1:
    th1_dot = -.1
print th1_dot



 
