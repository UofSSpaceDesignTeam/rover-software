from ADS1x15 import ADS1x15
from Adafruit_I2C import Adafruit_I2C
import time
#Really basic test for ADS1015 ADC. Reads an analog value on pin A0 and prints it.
#Note: When actuator is fully retracted the values are slightly off (approximately 1mm) 
ADS1015=0x00
adc = ADS1x15()
#actuator lengths
fullIn=292.354
fullOut=444.754
#raw data values
fullInR=1890
fullOutR=3081
while True:
	result = adc.readADCSingleEnded(2)
	#map the result to the range 0->1
	result=(result-fullInR)/(fullOutR-fullInR)
	#now map to the range fullIn -> fullOut
	result=result*(fullOut - fullIn) + fullIn
	#Scaling factor of 0.003.  12bits=4096 different states at a range of -6.144V to 6.144V gives 0.003V per bit
	print "Channel 1 = %.3f mm" % (result)
	time.sleep(0.010)
