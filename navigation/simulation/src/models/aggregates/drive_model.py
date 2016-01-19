from abc import abstractmethod
from models.aggregates import AggregateModel
from math import sqrt, cos, pi, sin, asin, isnan, atan2
from entities.coordinate import Coordinate
from random import gauss

from copy import deepcopy


class DriveModel(AggregateModel):
    """ Aggregate model to emulate the driving of the rover.
    
    Attributes:
        MAX_SPEED(float) -- The max speed the rover can drive (ie the speed
                            of the rover when power is full.
    Methods:
        update(timeStep) -- Update the position of the rover after `timeStep`
                            has elapsed.
    """
    
    MAX_SPEED = 1
    RKM = None
    
    def __init__(self, roverModel):
        AggregateModel.__init__(self, roverModel)
        
        self.roverProperties.powerLeft = 1
        self.roverProperties.powerRight = 1
        
        # Initialize RKM
        KMAInitial = KinematicModel.KMAInit()
        RDPInitial = KinematicModel.RDPInit()
        RFKInitial = KinematicModel.RFKInit()
        
        
        # Initialize Kinematic Model Attribute's
        KMAInitial.icr_Vx = 0.0
        KMAInitial.icr_Lx = 0.3
        KMAInitial.icr_Rx = 0.3
        KMAInitial.icr_Vy = 0.05
        KMAInitial.icr_Ly = 0.05
        KMAInitial.icr_Ry = 0.05
        KMAInitial.alpha_L = 0.91
        KMAInitial.alpha_R = 0.92
        
        # Initialize Rover Drive Properties 
        RDPInitial.v_L = 0.9
        RDPInitial.v_R = 0.9
        RDPInitial.position = self.roverProperties.position
        RDPInitial.heading = self.roverProperties.heading
        
        # Initialize Relative Frame Kinematics
        RFKInitial.vx = 0.01
        RFKInitial.vy = 0.89
        RFKInitial.wz = 0.001
        
        self.RKM = KinematicModel(KMAInitial, RDPInitial, RFKInitial)
        
        
    @abstractmethod
    def update(self, timeStep):
        """ Update the position of the rover after `timeStep` has elapsed.
        
        Pre:
            `roverProperties.position`    The current position of the
                                           rover.
            `roverProperties.powerLeft`   The power to the left side.
            `roverProperties.powerRight`   The power to the right side.
        Post:
            The position of the rover has been updated.
        Return:
            n/a
        """
        # Get left and right wheel velocities
        v_L = self.roverProperties.powerLeft * self.MAX_SPEED
        v_R = self.roverProperties.powerRight * self.MAX_SPEED
        
        self.RKM.update(v_L, v_R, timeStep)
        
        

  
class KinematicModel(object):
    # Relative frame kinematics.
    vx = None
    vy = None
    wz = None
    
    # Rover drive properties.
    v_L = None
    v_R = None
    
    # Kinematic model attributes.
    icr_Vx = None
    icr_Lx = None
    icr_Rx = None
    icr_Vy = None
    icr_Ly = None
    icr_Ry = None
    
    alpha_R = None
    alpha_L = None
    
    # Rover position attributes.
    position = None
    heading = None
    
    # Model archive.
    archive = None
    
    def __init__(self, KMAInitial, RDPInitial, RFKInitial):
        # Initialize relative frame kinematics.
        self.vx = RFKInitial.vx
        self.vy = RFKInitial.vy
        self.wz = RFKInitial.wz
        
        # Initialize drive properties.
        self.v_L = RDPInitial.v_L
        self.v_R = RDPInitial.v_R
        self.position = RDPInitial.position
        self.heading = RDPInitial.heading
        
        # Initialize kinematic model attributes.
        self.icr_Vx = KMAInitial.icr_Vx
        self.icr_Lx = KMAInitial.icr_Lx
        self.icr_Rx = KMAInitial.icr_Rx
    
        self.icr_Vy = KMAInitial.icr_Vy
        self.icr_Ly = KMAInitial.icr_Ly
        self.icr_Ry = KMAInitial.icr_Ry
        
        self.alpha_L = KMAInitial.alpha_L
        self.alpha_R = KMAInitial.alpha_R
    
        # Initialize archive.
        self.archive = KinematicModel.Archive()
        
    def archiveState(self):
        # Archive RFK
        self.archive.vx.append(self.vx)
        self.archive.vy.append(self.vy)
        self.archive.wz.append(self.wz)
        
        # Archive RDP
        self.archive.v_L.append(self.v_L)
        self.archive.v_R.append(self.v_R)
        
        # Archive KMA
        self.archive.icr_Vx.append(self.icr_Vx)
        self.archive.icr_Lx.append(self.icr_Lx)
        self.archive.icr_Rx.append(self.icr_Rx)
        self.archive.icr_Vy.append(self.icr_Vy)
        self.archive.icr_Ly.append(self.icr_Ly)
        self.archive.icr_Ry.append(self.icr_Ry)
        self.archive.alpha_L.append(self.alpha_L)
        self.archive.alpha_R.append(self.alpha_R)
        self.archive.position.append(deepcopy(self.position))
        self.archive.heading.append(deepcopy(self.position))
        
    def recalculateKMA(self):
        # Calculate new KMA
        self.icr_Vx = -self.vy/self.wz
        self.icr_Lx = (self.alpha_L*self.v_L - self.vy)/self.wz
        self.icr_Rx = (self.alpha_R*self.v_R - self.vy)/self.wz
        self.icr_Vy = self.vx/self.wz
        self.icr_Ly = self.vx/self.wz
        self.icr_Ry = self.vx/self.wz
    
    def recalculateRFK(self):
        # Calculate new RFK values
        coeff = 1.0/(self.icr_Rx - self.icr_Lx)
        self.vx = coeff * self.icr_Vy * ( -self.alpha_L*self.v_L + self.alpha_R*self.v_R)
        self.vy = coeff * ( self.icr_Rx*self.alpha_L*self.v_L - self.icr_Lx*self.alpha_R*self.v_R)
        self.wz = coeff * ( -self.alpha_L*self.v_L + self.alpha_R*self.v_R)
    
    def updatePosition(self, dt):
        v = sqrt(self.vx*self.vx + self.vy*self.vy) # current velocity
        distance = v * dt                           # distance moved in dt
        moveBearing = atan2(self.vx, self.vy)       # the bearing of the movement
        
        # update position and heading
        self.position = Coordinate.shiftCoordinate(self.position, moveBearing, distance)
        self.heading = self.heading + self.wz * dt
        
        
    
    def update(self, v_L, v_R, dt):
        self.recalculateKMA()
        self.recalculateRFK()
        self.updatePosition(dt)
        self.archiveState()
    
    class Archive(object):
        # Relative frame kinematics.
        vx = []
        vy = []
        wz = []
        
        # Rover drive properties.
        v_L = []
        v_R = []
        
        # Kinematic model attributes.
        icr_Vx = []
        icr_Lx = []
        icr_Rx = []
        icr_Vy = []
        icr_Ly = []
        icr_Ry = []
        alpha_R = []
        alpha_L = []
        
        # Rover position attributes.
        position = []
        heading  = []
        
        
    class KMAInit(object):
        # Initial conditions for kinematic model attributes.
        icr_Vx = None
        icr_Lx = None
        icr_Rx = None
        icr_Vy = None
        icr_Ly = None
        icr_Ry = None
        alpha_L = None
        alpha_R = None
        
    class RDPInit(object):
        # Initial conditions for rover drive properties.
        v_L = None
        v_R = None
        position = None
        heading = None
        
    class RFKInit(object):
        # Initial conditions for relative frame kinematics.
        vx = None
        vy = None
        wz = None
    
    
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
                
        
        
        
        
        