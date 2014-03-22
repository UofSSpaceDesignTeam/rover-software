from ADS1x15 import ADS1x15
from Adafruit_I2C import Adafruit_I2C
import time
#Really basic test for ADS1015 ADC. Reads an analog value on pin A0 and prints it.
ADS1015=0x00
adc = ADS1x15()
while True:
	result = adc.readADCSingleEnded(0)
	#Scaling factor of 0.003.  12bits=4096 different states at a range of -6.144V to 6.144V gives 0.003V per bit
	print "Channel 0 = %.3f V" % (result * 0.003)
	time.sleep(0.010)
