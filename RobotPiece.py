
# A class for a piecewise virtual representation of the robot.

	# dependency list
	
import pygame

	# class definition
	
class RobotPiece:
	def __init__(self, activeGraphic, inactiveGraphic, position):
		self.position = position
		self.inactiveGraphic = inactiveGraphic
		self.activeGraphic = activeGraphic
		self.active = False

	def draw(self, screen): # redraw the button with centered label
		if self.active:
			screen.blit(self.activeGraphic, self.position)
		else:
			screen.blit(self.inactiveGraphic, self.position)

