'''
Created on Nov 21, 2013

@author: caylin
'''
import pygame, pygame.font, pygame.event, pygame.draw, string, time
from pygame.locals import *
pygame.init()
'''
All object positions are defined relative to the screen size, so changing the screen
dimensions should scale everything
Question Box Locations:
Question 0:
Top Left Corner: ((screen.get_width() / 2) - 50, (screen.get_height() / 5) - 10)
Height:  24
Width:  150

Question 1:
Top Left Corner: ((screen.get_width() / 2) - 50, 2*(screen.get_height() / 5) - 10)
Height:  24
Width:  150

Question 2:
Top Left Corner: ((screen.get_width() / 2) - 50, 2*(screen.get_height() / 5) - 10)
Height:  24
Width:  150
'''
    
# sets up the initial display.  Shows the three titles and boxes
def displayInit(screen):
    fontobject = pygame.font.Font(None,18)
    #Sets up the screen the first time, displays titles. modify to change the displayed titles
    title = 'camera'
    for  x in xrange(3):
        screen.blit(fontobject.render(title, 1, (255,255,255)),
                        ((screen.get_width() / 2) - 100, (x+1)*(screen.get_height() / 5) - 10))
    #draw the rectangles
    pygame.draw.rect(screen, (255,255,255),
                   ((screen.get_width() / 2) - 50,
                    (screen.get_height() / 5) - 12,
                    150,24), 1)
    pygame.draw.rect(screen, (255,255,255),
                   ((screen.get_width() / 2) - 50,
                    2*(screen.get_height() / 5) - 12,
                    150,24), 1)
    pygame.draw.rect(screen, (255,255,255),
                   ((screen.get_width() / 2) - 50,
                    3*(screen.get_height() / 5) - 12,
                    150,24), 1)
    pygame.display.flip()

#get_key takes the screen and the number of the active user entry box and returns the character entered
def get_key(screen, userBox):
    circleColour = 0 #used to alternate the blinking indicator circle
    while 1:
        currentTime = int(2 * time.clock()) #currentTime is used to blink the indicator
        if int(currentTime/2.0) == (currentTime/2.0):
            if circleColour == 0:
                pygame.draw.circle(screen, (255,255,255), (((screen.get_width() / 2) - 112),((userBox+1)*(screen.get_height() / 5) - 5)), 4, 0)
                circleColour = 1
                pygame.display.flip()
        else:
            if circleColour == 1:
                pygame.draw.circle(screen, (0,0,0), (((screen.get_width() / 2) - 112),((userBox+1)*(screen.get_height() / 5) - 5)), 4, 0)
                circleColour = 0
                pygame.display.flip()
        for event in pygame.event.get():
            if event.type == MOUSEBUTTONDOWN: #if user left clicks it returns an arbitrary value indicating that
                if event.button == 1:
                    pygame.draw.circle(screen, (0,0,0), (((screen.get_width() / 2) - 112),((userBox+1)*(screen.get_height() / 5) - 5)), 4, 0)
                    return (900)
            if event.type == KEYDOWN: #if the user types it returns the associated number
                return (event.key)

#display_box takes the screen, the string entered to display, and the number for the active user box
def display_box(screen, message, userBox):
    fontobject = pygame.font.Font(None,18)
    #draw the rectangles
    pygame.draw.rect(screen, (0,0,0),
                   ((screen.get_width() / 2) - 50,
                    (userBox + 1)*(screen.get_height() / 5) - 10,
                    200,20), 0)
    pygame.draw.rect(screen, (255,255,255),
                   ((screen.get_width() / 2) - 50,
                    (userBox +1 ) * (screen.get_height() / 5) - 12,
                    150,24), 1)

    #draw the users text that has been entered
    screen.blit(fontobject.render(message, 1, (255,255,255)),
            ((screen.get_width() / 2) - 50, (userBox+1)*(screen.get_height() / 5) - 10))
    pygame.display.flip()

# ask takes the screen and returns an array of 3 strings.  It calls the functions that demand user interaction
def ask(screen):
    pygame.font.init()
    userClick = 900 #define an arbitrary value to indicate mouse clicks instead of typing
    userBoxes = [[[]for x in xrange(4)] for x in xrange(3)]
    for x in xrange(3): #define the dimensions of the displayed boxes
        userBoxes[x][0] = (screen.get_width() / 2) - 50
        userBoxes[x][1] = (x+1) * (screen.get_height() / 5) - 10 
    for x in xrange(3):
        userBoxes[x][2] = 150
        userBoxes[x][3] = 24
    ''' userBoxes[box number][parameter]
    Parameters:
    0 holds left X
    1 holds top Y
    2 holds width
    3 holds height '''
    time.clock() #initialize clock for blinking the indicator
    mousePos = [] #used to check if the user click occurs within a box
    current_string = [[] for x in xrange(3)] #keeps track of the entered text to return at the end
    userBox = 0 #tracks which box is currently active (0, 1, or 2)
    displayInit(screen) #passed to display_box to set up the title displays the first time
    display_box(screen, string.join(current_string[userBox],""), userBox)
    while 1:
        inkey = get_key(screen, userBox) #inkey holds the number representing the entered key
        if inkey == K_ESCAPE: #program quits if the user presses ESC
            pygame.display.quit()
            break
        if inkey == K_BACKSPACE: #deletes the previous character if the user enters backspace
            current_string[userBox]= current_string[userBox][0:-1]
        elif inkey == K_RETURN: #advances to next box if user presses enter, ends after the third
            pygame.draw.circle(screen, (0,0,0), (((screen.get_width() / 2) - 112),((userBox+1)*(screen.get_height() / 5) - 5)), 4, 0)
            userBox += 1
            if userBox == 3:
                return current_string
        elif inkey <= 127: #adds the cooresponding letter to the string
            current_string[userBox].append(chr(inkey))
        elif inkey == userClick: #if the user left clicks the mouse it will change to the box clicked on
            mousePos = pygame.mouse.get_pos()
            for x in xrange(3):
                if mousePos[0] >= userBoxes[x][0] and mousePos[0] <= (userBoxes[x][0] + userBoxes[x][2]):
                    if mousePos[1] >= userBoxes[x][1] and mousePos[1] <= (userBoxes[x][1] + userBoxes[x][3]):
                        userBox = x
        display_box(screen, string.join(current_string[userBox],""), userBox)
    return current_string

screen = pygame.display.set_mode((324,240)) #initialize screen
answer = ask(screen) #call the function
print answer
print answer[0]
print answer[1]
print answer[2]
