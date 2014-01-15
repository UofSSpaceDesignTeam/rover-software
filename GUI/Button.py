import pygame
pygame.init()

class Button:
   def __init__(self, text):
      self.text = text
      self.default_color = (100,100,100)
      self.selected_color = (0,0,0)
      self.font_color = (0,0,0)
      self.font_size = 20
      self.selected = False
      self.obj = None
      
   def label(self):
      '''button label font'''
      font = pygame.font.Font(None, self.font_size)
      return font.render(self.text, 1, self.font_color)
      
   def color(self):
      '''change color when selected'''
      if self.selected:
	  return self.selected_color
      else:
	  return self.default_color
         
   def draw(self, screen, mouse, rectcoord, labelcoord):
      '''create rect obj, draw, and change color based on input'''
      self.obj  = pygame.draw.rect(screen, self.color(), rectcoord)
      screen.blit(self.label(), labelcoord)
