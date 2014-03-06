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
class VirtualRobot():


	def __init__(self,left,top, width,height):
		self.left = left
		self.top = top
		self.width = width
		self.height = height
		self.body = (left,top,width,height)
		self.color = colorGray
		self.sColor = colorYellow
		self.armCol = colorBlue
		self.armCamCol = colorRed
		self.driveCamCol = colorRed
		self.arm = (left+width/2,top-height/4,width/4,height/4)
		self.armCam = (left+width/2+width/16,top-height/4,width/8,width/8)
		self.driveCam = (left+width/2-width/16,top +height/2,width/8,width/8)
		self.driveWheel = pygame.image.load("./graphics/redwheel.png")
		self.drivecamera = pygame.image.load("./graphics/rcamera.png")
		self.armObj = None
		self.armCamObj = None
		self.bodyObj = None
		self.driveCamObj = None

	def draw(self,screen):
		self.bodyObj = pygame.draw.rect(screen,self.color,self.body)
		self.armObj = pygame.draw.rect(screen, self.armCol, self.arm)
		screen.blit(self.driveWheel,(self.left,self.top))
		screen.blit(self.driveWheel,(self.left,self.top + self.height - 30))
		screen.blit(self.driveWheel,(self.left + self.width - 15, self.top))
		screen.blit(self.driveWheel,(self.left+self.width -15, self.top + self.height - 30))
		screen.blit(self.driveWheel, (self.left, self.top + self.height/2))
		screen.blit(self.driveWheel,(self.left + self.width-15, self.top+self.height/2))
		screen.blit(self.drivecamera,(self.left+self.width/2-self.width/16,self.top+self.height/2))
		self.armCamObj = pygame.draw.rect(screen,self.armCamCol, self.armCam)

	def turnOnCamera(self, i):
		if(i == 0):
			self.driveCamCol = colorGreen
		elif(i == 1):
			self.armCamCol = colorGreen
			
	def turnOffCamera(self, i):
		if(i == 0):
			self.driveCamCol = colorRed
		elif(i == 1):
			self.armCamCol = colorRed
	
	def wheelsConnected(self):
		self.driveWheel = pygame.image.load("./graphics/blackwheel.png")
		
	def wheelsDisconnected(self):
		self.driveWheel = pygame.image.load("./graphics/redwheel.png")