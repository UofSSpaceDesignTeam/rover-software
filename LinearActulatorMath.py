# math to calculate the linear actuator speed and length

import cmath

# constants, in mm 
A1 = 372.0
B1 = 105.0
A2 = 121.0
B2 = 363.0


# values for variables have to be added
th1 = 2
L1 = 300.0        # has to be between 283.5  and 446.5 mm 
th2 = 2
L2 = 390.0	    # has to between 380.8 and 152.4 mm 

# L1p,L2p is speed of the the actuator 
# L1,L2 is position of actuator 
# th1,1h2 (theta) is the angular speed 



L1p = (th1*A1*B1)/L1 * cmath.sqrt( (1 - pow( ( (pow(A1,2)+pow(B1,2)-pow(L1,2))/(2*A1*B1)),2) ) ) 
print L1p

L2p = (th2*A2*B2)/L2 * cmath.sqrt( (1 - pow( ((pow(A2,2)+pow(B2,2)-pow(L2,2))/2*A2*B2),2) ) ) 
print L2p

