
# A class for indicator lights in the GUI.
# Added by Jordan

	# dependency list
	
import pygame


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
		self.font = pygame.font.Font(None, 24)


	def getColor(self):	# returns the color of the button (3-tuple)
		if self.active:
			return self.activeColor
		else:
			return self.defaultColor


	def draw(self, screen): # redraw the button with centered label
		pygame.draw.rect(screen, (0, 0, 0), (self.position[0] - 3, self.position[1] - 3, 31, 31))
		pygame.draw.rect(screen, self.getColor(), (self.position[0], self.position[1], 25, 25))
		screen.blit(self.font.render(self.text, 1, self.fontColor), (self.position[0] + 34, self.position[1] + 6))


	def refresh(self):	# call boolean function to determine state
		self.active = self.function(self.args)

