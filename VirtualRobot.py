import pygame

colorWhite = (255, 255, 255)
colorGray = (125, 125, 125)
colorBlack = (0, 0, 0)
colorRed = (240, 0, 0)
colorGreen = (0, 240, 0)
colorDarkGreen = (0, 100, 0)
colorBlue = (0, 0, 240)
colorDarkBlue = (0, 0, 120)
colorYellow = (250, 250, 0)
class virtualRobot():


	def __init__(self,left,top, width,height):
		self.left = left
		self.top = top
		self.width = width
		self.height = height
		self.body = (left,top,width,height)
		self.color = colorGray
		self.sColor = colorYellow
		self.driveCol = colorRed
		self.armCol = colorRed
		self.armCamCol = colorRed
		self.driveCamCol = colorRed
		self.arm = (left+width/2,top-height/4,width/4,height/4)
		self.armCam = (left+width/2-width/8,top-height/4,width/8,width/8)
		self.driveCam = (left+width/2-width/8,top +height/2,width/8,width/8)
		self.driveWheel1 = (left,top,width/8,height)
		self.driveWheel2 = (left+width*7/8,top,width/8,height)
		self.driveWheel1Obj= None
		self.driveWheel2Obj=None
		self.armObj = None
		self.armCamObj = None
		self.bodyObj = None
		self.driveCamObj = None

	def draw(self,screen):
		self.armObj = pygame.draw.rect(screen, self.armCol, self.arm)
		self.armCamObj = pygame.draw.rect(screen,self.armCamCol, self.armCam)
		self.bodyObj = pygame.draw.rect(screen,self.color,self.body)
		self.driveWheel1Obj = pygame.draw.rect(screen,self.driveCol,self.driveWheel1)
		self.driveWheel2Obj = pygame.draw.rect(screen,self.driveCol,self.driveWheel2)
		self.driveCamObj = pygame.draw.rect(screen,self.driveCamCol,self.driveCam)
		