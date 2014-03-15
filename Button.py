
# A class for clickable buttons in the GUI.
# Added by Dylan

	# dependency list
	
import pygame


	# class definition
	
class Button:
	def __init__(self, function, (args), text, fontSize, fontColor, rect, defaultColor, selectedColor):
		self.function = function
		self.args = args
		self.text = text
		self.fontSize = fontSize
		self.fontColor = fontColor
		self.rect = rect
		self.defaultColor = defaultColor
		self.selectedColor = selectedColor
		self.selected = False
		self.obj = None

	def renderLabel(self):	# returns a Surface with the button label on it
		font = pygame.font.Font(None, self.fontSize)
		return font.render(self.text, 1, self.fontColor)

	def getColor(self):	# returns the color of the button (3-tuple)
		if self.selected:
			return self.selectedColor
		else:
			return self.defaultColor

	def draw(self, screen): # redraw the button with centered label
		label = self.renderLabel()
		labelSize = label.get_size()
		labelXPosition = self.rect[0] + 0.5 * (self.rect[2] - labelSize[0])
		labelYPosition = self.rect[1] + 0.5 * (self.rect[3] - labelSize[1])
		self.obj = pygame.draw.rect(screen, self.getColor(), self.rect)
		screen.blit(label, (labelXPosition, labelYPosition))

	def press(self):	# activate the button
		self.function(self.args)

