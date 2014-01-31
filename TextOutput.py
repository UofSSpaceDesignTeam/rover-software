import pygame
import time

class TextOutput():
	
	def __init__(self,fontsize,fontcolor, left, top, width, height,arraySize,color):
		self.textSize = fontsize
		self.height = height
		self.rect = (left,top,width,height)
		self.innerRect = (left+2,top+2,width-4,height-4)
		self.textStartleft = left + fontsize/10
		self.textStarttop = top +fontsize/10
		self.maxArraySize = arraySize +1
		self.vOffset = fontsize + fontsize/10
		self.list = []
		self.color = color
		self.textcolor = fontcolor
		self.obj1 = None
		self.obj2 = None
	
	def write(self,text):
		text.rstrip()
		if len(text) < 5: return
		self.list.append(text)
		if(len(self.list) >= self.maxArraySize):
			self.list.remove(self.list[0])

	def renderText(self, counter):
		font = pygame.font.Font(None, self.textSize)
		print(counter)
		return font.render(self.list[counter], 1, self.textcolor)

	def draw(self,screen):
		self.obj1 = pygame.draw.rect(screen, self.color, self.rect)
		self.obj2 = pygame.draw.rect(screen,(0,0,0,0),self.innerRect)
		i=len(self.list)
		offset = i -len(self.list)-1
		while( i>0):
			i = i-1
			screen.blit(self.renderText(i), (self.textStartleft,self.textStarttop-self.vOffset-(offset)*self.vOffset))
			offset=offset-1


