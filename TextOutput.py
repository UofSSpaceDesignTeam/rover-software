import pygame
import time

class TextOutput():
	
	def __init__(self,fontsize,fontcolor, left, top, width, height,arraySize,color):
		self.textSize = fontsize
		self.height = height
		self.rect = (left,top,width,height)
		self.innerRect = (left+2,top+2,width-4,height-4)
		self.textStartleft = left + fontsize/20
		self.textStarttop = top +fontsize/20
		self.maxArraySize = arraySize +1
		self.hOffset = fontsize + fontsize/10
		self.vOffset = fontsize + fontsize/10
		self.list = []
		self.color = color
		self.textcolor = fontcolor
		self.obj1 = None
		self.obj2 = None
	
	def write(self,text):
		if len(text) < 3: return
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
		offset=len(self.list)
		i=0
		offset = offset -len(self.list)-1
		while( i<len(self.list)):
			screen.blit(self.renderText(i), (self.textStartleft + self.hOffset,self.textStarttop-self.vOffset-(offset)*self.vOffset))
			i = i+1
			offset=offset-1


