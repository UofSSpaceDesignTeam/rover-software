from .RoverProcess import RoverProcess

# Any libraries you need can be imported here. You almost always need time!
import time


class StateManagerTestProcess2(RoverProcess):
	def getSubscribed(self):
		return ["Test"]
	
	def messageTrigger(self, message):
		
		if "Test" in message:
			print("Process 2 got: " + str(message["Test"]))
	
	