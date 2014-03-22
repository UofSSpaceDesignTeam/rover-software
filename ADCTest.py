from Adafruit_ADS1x15 import ADS1x15
import time

ADS_Current = ADS1115
adc = ADS1x15(ic=ADS_Current)
while True:
	result = adc.readADCSingleEnded(0)
	print "Channel 0 = %.3f V" % (result * 0.003)
	time.sleep(0.010)