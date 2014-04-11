	
import pygame
	
class Box:

	def __init__(self, label, rect, labelPosition):
		self.label = label
		self.rect = rect
		self.labelPosition = labelPosition
		self.font = pygame.font.Font(None, 22)

	def draw(self, screen): # redraw the box and the label text
		pygame.draw.rect(screen, (100, 100, 100), self.rect)
		screen.blit(self.font.render(self.label, 1, (255, 255, 255)), self.labelPosition)

