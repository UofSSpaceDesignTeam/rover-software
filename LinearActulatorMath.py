# math to calculate the linear actuator speed and length

import math

# constants, in mm 
A1 = 372
B1 = 105
A2 = 121
B2 = 363


# values for variables have to be added
th1 = 
L1 = 
th2 = 
L2 = 

# L1p,L2p is speed of the the actuator 
# L1,L2 is position of actuator 
# th1,1h2 (theta) is the angular speed 



L1p = (th1*A1*B1)/L1 * math.sqrt( (1 - pow( ( (pow(A1,2)+pow(B1,2)-pow(L1,2))/2*A1*B1),2) ) ) 
print L1p

L2p = (th2*A2*B2)/L2 * math.sqrt( (1 - pow( ((pow(A2,2)+pow(B2,2)-pow(L2,2))/2*A2*B2),2) ) )
print L2p

