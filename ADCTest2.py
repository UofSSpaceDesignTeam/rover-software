from ADS1x15 import ADS1x15
from Adafruit_I2C import Adafruit_I2C
import time
import smbus

# Test fot the ADC for development of the experiment...

adc = ADS1x15(0x4B) # Address is GND->SCL (0x4B)
gain = 256 # +/- .256V
sps = 8 # sample rate

while True:
	volt = adc.readADCSingleEnded(0, gain, sps)/1000
	#map the result to the range 0->1
	print "Channel A0 = %.3f V" % (volt)
	time.sleep(0.010)
