# A basic test GUI for the XBox 360 Controller
# based on an example from programarcadegames.com
# Added by Kyle Bonilla 


# Import a library of functions called 'pygame'
import pygame
import Controller 
 
# Initialize the game engine
pygame.init()

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)
BLUE     = (   0,   0, 255)
GREEN    = (   0, 255,   0)
RED      = ( 255,   0,   0)
YELLOW   = ( 225, 225,   0) 
 
# Set the height and width of the screen
size = (400, 500)
screen = pygame.display.set_mode(size)
 
pygame.display.set_caption("Py game testing")
 
#Loop until the user clicks the close button.
done = False
clock = pygame.time.Clock()

# sets rect varibles

rect_x = 175
rect_y = 200

colour = GREEN



# -------------------------------------------
# main loop

# Loop as long as done == False
# allows window to be closed 
while not done:
 
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done = True # Flag that we are done so we exit this loop

    # --------------------------------------------------------               
    # logic bellow comment


    Controller.init()


    axes = Controller.getAxes()
    roundedAxes = [round(i,2) for i in axes]

    buttons = Controller.getButtons()


    if buttons[0]:
        colour = GREEN

    if buttons[1]:
        colour = RED

    if buttons[2]:
        colour = BLUE 

    if buttons[3]:
        colour = YELLOW
    
	
   
    # Move the rectangle starting point
    rect_x += roundedAxes[0]*5
    rect_y += -roundedAxes[1]*5

    # stops the rectangle 
    if rect_y > 450:
        rect_y = 450
    if rect_y < 0:
        rect_y = 0
		
    if rect_x > 350:
        rect_x = 350
    if rect_x < 0:
        rect_x = 0
        


    # logic above comment
    #--------------------------------------------------------
  
    # Clear the screen and set the screen background
    screen.fill(WHITE)

    # drawing below comment

 
    # Draw a rectangle
    pygame.draw.rect(screen, colour , [rect_x, rect_y, 50, 50])


    #Text
    # Select the font to use. Default font, 25 pt size.
    font = pygame.font.Font(None, 25)

    # forms the text, but does not print it
    # True means anti-aliased, BLACK is colour
    xtext = font.render("x-axis: " + str( roundedAxes[0] ), True, BLACK)
    ytext = font.render("y-axis: " + str( roundedAxes[1]), True, BLACK)
    ctext = font.render("colour: " + str( colour), True, BLACK)
 
    #put text on screen
    screen.blit(xtext, [10,400])
    screen.blit(ytext, [10,425])
    screen.blit(ctext, [10,450])
    
  
    # drawing above comment
    # ---------------------------------------------------------
    # update the screen with what we've drawn.
    # This MUST happen after all the other drawing commands.
    pygame.display.flip()
 
    # This limits the while loop to a max of 60 times per second.
    # Leave this out and we will use all CPU we can.
    clock.tick(60)
     
 
# Be IDLE friendly
pygame.quit()
