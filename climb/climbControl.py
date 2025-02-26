import wpilib
from utils.constants import CLIMB_CANID, FUNNEL_SERVO
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class ClimbControl:
    def __init__(self):
        self.climbMotor = WrapperedSparkMax(CLIMB_CANID, "ClimbMot", True)
        self.servo = wpilib.Servo(FUNNEL_SERVO)
        self.climbMotor.setInverted(False)
        self.cmdVolt = 0
        self.cmdPos = 0

    def update(self):
        self.climbMotor.setVoltage(self.cmdVolt)
        self.servo.set(self.cmdPos)

    def setClimbCmdVolt(self, voltage):
        self.cmdVolt = voltage

    def setServoCmdPos(self, poseBool):
        if poseBool:
            self.cmdPos = 1
        else:
            self.cmdPos = 0