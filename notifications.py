import pygame


class Notification:
	def __init__(self, top,left,bottom,right, msg,pos):
		self.position = pos
		self.message = msg
		self.length = 5
		self.color = (0, 0, 0)
		self.rect = (top,left,bottom,right)
		self.textcolor = (0, 240, 0)