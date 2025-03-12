import wpilib
from utils.constants import CLIMB_CANID
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class ClimbControl:
    def __init__(self):
        self.climbMotor = WrapperedSparkMax(CLIMB_CANID, "ClimbMot", brakeMode=True, currentLimitA=60)
        self.climbMotor.setInverted(False)
        self.cmdVolt = 0
        self.cmdPos = 0

    def update(self):
        self.climbMotor.setVoltage(self.cmdVolt)

    def setClimbCmdVolt(self, voltage):
        self.cmdVolt = voltage