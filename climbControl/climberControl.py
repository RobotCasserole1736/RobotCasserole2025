
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from utils.constants import CLIMB_CANID

class ClimberControl:
    def __init__(self):
        self.climbMotor = WrapperedSparkMax(CLIMB_CANID, "ClimbMotor", True)
        self.cmdVel = 0

    def update(self):
        self.climbMotor.setVoltage(self.cmdVel)

    def setClimbVol(self, climbVel):
        self.cmdVel = climbVel