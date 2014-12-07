import config.baseMessages
import threading
import json
import pygame
from Queue import Queue
import time
import unicodeConvert

convert = unicodeConvert.convert

class inputThread(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.name = "inputThread"
		self.commThread = None
		self.exit = False
		self.mailbox = Queue()
		self.cont = [False, False]
		pygame.init()
		if pygame.joystick.get_count() > 0:
			self.cont[0] = pygame.joystick.Joystick(0)
			self.cont[0].init()
		if pygame.joystick.get_count() > 1:
			self.cont[1] = pygame.joystick.Joystick(1)
			self.cont[1].init()

	def run(self):
		while not self.exit:
			msg = {}
			pygame.event.pump()
			if self.cont[0]:
				# uncomment messages as needed.
				msg["c1t"] = self.filter(self.cont[0].get_axis(2) * -1)
				# msg["c1j1x"] = self.filter(self.cont[0].get_axis(0))
				# msg["c1j1y"] = self.filter(self.cont[0].get_axis(1) * -1)
				# msg["c1j2x"] = self.filter(self.cont[0].get_axis(4))
				# msg["c1j2y"] = self.filter(self.cont[0].get_axis(3) * -1)
				# msg["c1b_a"] = self.cont[0].get_button(0)
				# msg["c1b_b"] = self.cont[0].get_button(1)
				# msg["c1b_x"] = self.cont[0].get_button(2)
				# msg["c1b_y"] = self.cont[0].get_button(3)
				# msg["c1b_lb"] = self.cont[0].get_button(4)
				# msg["c1b_rb"] = self.cont[0].get_button(5)
				# msg["c1b_ba"] = self.cont[0].get_button(6)
				# msg["c1b_st"] = self.cont[0].get_button(7)
				# msg["c1d_x"] = self.cont[0].get_hat(0)[0]
				# msg["c1d_y"] = self.cont[0].get_hat(0)[1]
			# if self.cont[1]:
				# uncomment messages as needed.
				# msg["c2t"] = self.filter(self.cont[1].get_axis(2) * -1)
				# msg["c2j1x"] = self.filter(self.cont[1].get_axis(0))
				# msg["c2j1y"] = self.filter(self.cont[1].get_axis(1) * -1)
				# msg["c2j2x"] = self.filter(self.cont[1].get_axis(4))
				# msg["c2j2y"] = self.filter(self.cont[1].get_axis(3) * -1)
				# msg["c2b_a"] = self.cont[1].get_button(0)
				# msg["c2b_b"] = self.cont[1].get_button(1)
				# msg["c2b_x"] = self.cont[1].get_button(2)
				# msg["c2b_y"] = self.cont[1].get_button(3)
				# msg["c2b_lb"] = self.cont[1].get_button(4)
				# msg["c2b_rb"] = self.cont[1].get_button(5)
				# msg["c2b_ba"] = self.cont[1].get_button(6)
				# msg["c2b_st"] = self.cont[1].get_button(7)
				# msg["c2d_x"] = self.cont[1].get_hat(0)[0]
				# msg["c2d_y"] = self.cont[1].get_hat(0)[1]	
			if self.cont[0] or self.cont[1]:
				self.commThread.mailbox.put(msg)
				self.mailbox.put(msg)
			time.sleep(0.2)

	def stop(self):
		self.exit = True
	
	def filter(self, value):
		if abs(value) < 0.15:
			return 0.0
		elif value > 1.0:
			return 1.0
		elif value < -1.0:
			return -1.0
		return value