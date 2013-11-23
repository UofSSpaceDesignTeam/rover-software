import pygame

class TextBox:
    def __init__(self, rect, width=1):
        self.selected = False
        self.font_size = 15
        self.font = pygame.font.SysFont('Arial', self.font_size)
        self.str_list = []
        self.width = width
        self.color = (0,0,0)
        self.rect = rect
        self.string = ''.join(self.str_list)
        
    def char_add(self, event):
        '''modify string list based on event.key'''
        if event.key == pygame.K_BACKSPACE:
            if self.str_list:
                self.str_list.pop()
        elif event.key == pygame.K_RETURN:
            return ''.join(self.str_list)
        elif event.key in [pygame.K_TAB, pygame.K_KP_ENTER]:#unwanted keys
            return False
        elif event.key == pygame.K_DELETE:
            self.str_list = []
            return False
        else:
            char = event.unicode
            if char: #stop emtpy space for shift key adding to list
                self.str_list.append(char)

    def update(self, scr):
        
        if self.selected:
            width = 2
        else:
            width = self.width
        
        s = ''.join(self.str_list)
        if len(s) > 0:
            for n, l in enumerate(s):
                if self.font.size(s[n:])[0] < self.rect.width:
                    label = self.font.render(s[n:], 1, self.color)
                    break
        else:
            label = self.font.render(s, 1, self.color)
        
        self.string = ''.join(self.str_list)
        pygame.draw.rect(scr, self.color, self.rect, width)
        scr.blit(label, self.rect)
 