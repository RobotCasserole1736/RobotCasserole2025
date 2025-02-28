import wpilib
from utils.constants import CLIMB_CANID, FUNNEL_SERVO
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class ClimbControl:
    def __init__(self):
        self.climbMotor = WrapperedSparkMax(CLIMB_CANID, "ClimbMot", brakeMode=True, currentLimitA=60)
        self.servo = wpilib.Servo(FUNNEL_SERVO)
        self.climbMotor.setInverted(False)
        self.cmdVolt = 0
        self.cmdPos = 0
        self.servoWent = False

    def update(self):
        self.climbMotor.setVoltage(self.cmdVolt)
        #if not self.servoWent:
        self.servo.set(self.cmdPos)

        """if self.cmdPos == 1:
                #if the command position is not its default. 
                self.servoWent = True"""

    def setClimbCmdVolt(self, voltage):
        self.cmdVolt = voltage

    def setServoCmdPos(self, poseBool):
        if poseBool:
            self.cmdPos = 1
        else:
            self.cmdPos = .25