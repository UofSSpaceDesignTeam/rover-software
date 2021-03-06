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

# Any libraries you need can be imported here. You almost always need time!
import time


class ExampleProcess_Watchdog(RoverProcess):
	# Some blank space to write functions, classes, threads - whatever you need.
	# There are no restrictions - this is your own process!

	# See the tutorials on the wiki for things like basic threads and custom libraries.
	# < TO DO: INSTERT LINK TO THAT WIKI AND WRITE IT >

	# This is run once to set up anything you need.
	# Hint: use the self object to store variables global to this process.
	# You can also take any args you need from the main startup as a dictionary:
	# self.something = args["something"]
	# This can be handy for sharing semaphores with other processes!
	def setup(self, args):
		# Each RoverProcess instance has a member called "subscriptions"
		# which is a list of messages it is currently subscribed to.
		# You can read from this list, but please do not modify it,
		# as that will mess things up. Use subscribe() and unsubscribe().
		for key in ["Test", "respondTrue", "heartbeat"]:
			self.subscribe(key)
		self.someVariable = 42
		self.loopcounter = 0
		self.log("This is the setup of the watchdog test!")

	# This automatically loops forever.
	# It is best to put a time.sleep(x) at the end. This makes sure that it
	# will always run at the same time, and will give other processes time to run too!
	# The default behavior is to sleep for 1 second.
	# Use self.publish() to send some variables to another process or server!
	#
	# To Demonstrate watchdog functionality, see below
	def loop(self):
		if(self.loopcounter < 2):
			self.log("Process Normal Operation")
			time.sleep(4)
		elif(self.loopcounter < 4):
			self.log("Process Temporary Extend Watchdog")
			self.watchdogExtend(10)
			time.sleep(8)
			self.watchdogExtend('PREVIOUS')
		elif(self.loopcounter < 6):
			self.log("Process Manual Pet Watchdog in Loop")
			time.sleep(4)
			self.watchdogPet()
			time.sleep(4)
		elif(self.loopcounter < 8):
			self.log("Process Manual Watchdog State Clear")
			time.sleep(4)
			self.watchdogReset()
			time.sleep(4)
		elif(self.loopcounter < 10):
			self.log("Process Hang")
			time.sleep(12)

		self.loopcounter = self.loopcounter + 1


	# This runs every time a new message comes in.
	# It is often handy to have an if statement for every type of message you expect
	def messageTrigger(self, message):
		RoverProcess.messageTrigger(self, message)

		if message.key == 'Test':
			self.log("got: " + str(message.data))

	# This runs once at the end when the program shuts down.
	# You can use this to do something like stop motors, clean up open files, etc.
	def cleanup(self):
		# If you override this method, you must call RoverProcess.cleanup(self)
		RoverProcess.cleanup(self)

	# Whenever the "heartbeat" message is produced, the callback function "on_heartbeat"
	# is called. This is an alternative to using large if/else structures in
	# messageTrigger.
	def on_heartbeat(self, data):
		self.log("From callback got: " + str(data))

	# This method demonstrates the testing framework. If test_ExampleProcess
	# is running, it will send the "respondTrue" message with a value of False
	# This module must respond True to pass the test
	def on_respondTrue(self, data):
		self.publish("response", not data)
