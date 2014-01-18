
# A class for labeled graphical boxes
# Added by Jordan

	# dependency list
	
from pygame import font, Surface


	# class definition
	
class Box:

	def __init__(self, text, fontSize, fontColor, rect, textPosition, color):
		self.text = text
		self.fontSize = fontSize
		self.fontColor = fontColor
		self.rect = rect
		self.textPosition = textPosition
		self.color = color
		self.obj = None


	def renderLabel(self):	# returns a Surface with the box label on it
		font = pygame.font.Font(None, self.fontSize)
		return font.render(self.text, 1, self.fontColor)


	def draw(self, screen): # redraw the box and the label text
		self.obj = pygame.draw.rect(screen, self.color, self.rect)
		screen.blit(self.renderLabel(), self.textPosition)

