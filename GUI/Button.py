
# A class for clickable buttons in the GUI.
# Added by Dylan

	# dependency list
	
from pygame import font, Surface


	# class definition
	
class Button:
	def __init__(self, text):	# object constructor
		self.text = text
		self.default_color = (100,100,100)
		self.selected_color = (0,0,0)
		self.font_color = (0,0,0)
		self.font_size = 20
		self.selected = False
		self.obj = None


	def label(self):	# returns a Surface with the button label on it
		font = pygame.font.Font(None, self.font_size)
		return font.render(self.text, 1, self.font_color, self.color())


	def color(self):	# returns the color of the button (3-tuple)
		if self.selected:
			return self.selected_color
		else:
			return self.default_color


	def draw(self, screen, mouse, rectcoord, labelcoord): # redraw the button
		self.obj = pygame.draw.rect(screen, self.color(), rectcoord)
		screen.blit(self.label(), labelcoord)

