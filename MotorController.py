import serial
import time
import pygame

motor = serial.Serial("/dev/ttyAMA0")
motor.baudrate=9600
motor.timeout=3

motor.write("1,start\r\n")
motor.write("1,home\r\n")

time.sleep(.5)

pygame.init()

colorWhite = (255, 255, 255)
colorGray = (125, 125, 125)
colorBlack = (0, 0, 0)
colorRed = (240, 0, 0)
colorGreen = (0, 240, 0)
colorBlue = (0, 0, 240)
colorYellow = (250, 250, 0)

size = (400, 500)
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Motor Controller Display")

done = False
clock = pygame.time.Clock()

pygame.display.update()

timespan = 0;

rect_x = 88
rect_y = 50

while not done:
 
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done = True # Flag that we are done so we exit this loop

    # logic
	if timespan < 600:
		if timespan == 0:
			motor.write("1,s500\r\n")
		motor.write("1,getp\r\n")
		position = motor.readline()
		motor.write("1,gets\r\n")
		speed = motor.readline()
		timespan += 1
	else:
		if timespan == 600:
			motor.write("1,s-500\r\n")
		motor.write("1,getp\r\n")
		position = motor.readline()
		motor.write("1,gets\r\n")
		speed = motor.readline()
		timespan += 1
		if timespan == 1200:
			timespan = 0

    # end logic        
    screen.fill(WHITE)
    pygame.draw.rect(screen, GREEN, [rect_x, rect_y, 50, position/150])
    font = pygame.font.Font(None, 25)
    xtext = font.render("speed: " + "10", True, BLACK)
    ytext = font.render("position: " + "10", True, BLACK)
    #put text on screen
    screen.blit(xtext, [10,400])
    screen.blit(ytext, [10,425]) 
    pygame.display.update()
    clock.tick(60)

pygame.quit()