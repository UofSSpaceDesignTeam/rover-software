# Copyright 2016 University of Saskatchewan Space Design Team Licensed under the
# Educational Community License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License. You may
# obtain a copy of the License at
#
# https://opensource.org/licenses/ecl2.php
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an "AS IS"
# BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
# or implied. See the License for the specific language governing
# permissions and limitations under the License.

from .RoverProcess import RoverProcess
import pyvesc

# Any libraries you need can be imported here. You almost always need time!
import time
from multiprocessing.synchronize import BoundedSemaphore # BoundedSemaphore class used to block extra commands from conflicting with actual commands in operation.

base_max_speed = 40000
base_min_speed = 11000

class ArmProcess(RoverProcess):
    
	# Subscribe ArmProcess to joystick keys for multiprocessing.
	def setup(self, args):
		for key in ["joystick1", "joystick2", "Rtrigger"]: # Add the keys to the subscription of the multiprocessor.
			self.subscribe(key)
		self.base_direction = None

	# Function that grabs the x and y axis values in message, then formats the data
	#  and prints the result to stdout.
	# Returns the newly formated x and y axis values in a new list
	def on_joystick1(self, data):
		x_axis = data[0] # Get data for x-axis.
		x_axis = (x_axis * base_max_speed/2) # Conversion factor to make it easy to interpret input, half power for testing.
		if x_axis > base_min_speed or x_axis < -11000: # If out of dead-zone.
			armBaseSpeed = x_axis
		else:
			armBaseSpeed = 0
		self.log(armBaseSpeed, "DEBUG") # Print the status of this process.
		self.publish("armBase", armBaseSpeed) # Publish the process to the multiprocessor.

	def on_Rtrigger(self, trigger):
		trigger = -1*(trigger + 1)/2
		armBaseSpeed = trigger * base_max_speed/2
		if self.base_direction is "right" or self.base_direction is None:
			if -base_min_speed <armBaseSpeed < base_min_speed:
				armBaseSpeed = 0
				self.base_direction = None
			else:
				self.base_direction = "right"
			self.log(armBaseSpeed, "DEBUG")
			self.publish("armBase", pyvesc.SetRPM(armBaseSpeed)) # Publish the process to the multiprocessor.


	def on_Ltrigger(self, trigger):
		trigger = (trigger + 1)/2
		armBaseSpeed = trigger * base_max_speed/2
		if self.base_direction is "left" or self.base_direction is None:
			if -base_min_speed <armBaseSpeed < base_min_speed:
				armBaseSpeed = 0
				self.base_direction = None
			else:
				self.base_direction = "left"
			self.log(armBaseSpeed, "DEBUG")
			self.publish("armBase", pyvesc.SetRPM(armBaseSpeed)) # Publish the process to the multiprocessor.







