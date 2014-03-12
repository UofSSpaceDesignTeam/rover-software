
# A class for indicator lights in the GUI.
# Added by Jordan

	# dependency list
	
import pygame
import sys

	# class definition
	
class Indicator:

	def __init__(self, function, (args), text, fontColor, position, defaultColor, activeColor):
		self.function = function
		self.args = args
		self.text = text
		self.fontColor = fontColor
		self.position = position
		self.defaultColor = defaultColor
		self.activeColor = activeColor
		self.active = False
		self.font = pygame.font.Font(None, 20)


	def getColor(self):	# returns the color of the button (3-tuple)
		if self.active:
			return self.activeColor
		else:
			return self.defaultColor


	def draw(self, screen): # redraw the button with centered label
		pygame.draw.rect(screen, (10, 10, 10), (self.position[0] - 2, self.position[1] - 2, 18, 18))
		pygame.draw.rect(screen, self.getColor(), (self.position[0], self.position[1], 14, 14))
		screen.blit(self.font.render(self.text, 1, self.fontColor), (self.position[0] + 20, self.position[1] + 1))


	def refresh(self):	# call boolean function to determine state
		if self.function(self.args):
			self.active = True
		elif self.active:
			sys.stderr.write(self.text + " connection lost")
			self.active = False
		else:
			self.active = False