import wpilib
import wpimath 
from utils.singleton import Singleton
from enum import Enum 
import utils.constants
 
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class CoralManState(Enum):
    DISABLED = 0
    INTAKING = 1
    EJECTING = 2 
    HOLDING = 3

class CoralManipulatorControl(metaclass=Singleton):

    def __init__(self):
        self.coralCurState = CoralManState.DISABLED
        self.coralMotorL = WrapperedSparkMax(13, "CoralMotorL", True, 10)
        self.coralMotorR = WrapperedSparkMax(14, "CoralMotorR", True, 10)
        self.gamepieceSensorF = None #uhhh this one depends on what/how we do so its just gonna be empty inside for now.
        self.gamepieceSensorR = None #uhhh this one depends on what/how we do so its just gonna be empty inside for now.        
        self.curMotorVoltage = 0.0
        self.hasGamePiece = False
        self.motorHoldingSpeed = 1.0
        self.motorIntakeSpeed = 0.0
        self.motorEjectSpeed = 0.0

        pass 

    def update(self):

        self.hasGamePiece = self.checkGamePiece() #Check if we are holding a piece of coral. This way it will automatically switch to holding mode.
        if self.hasGamePiece:
            self.coralCurState = CoralManState.HOLDING
        
        if self.coralCurState != CoralManState.DISABLED:
            if self.coralCurState == CoralManState.INTAKING:
                self.coralMotorL.setVelCmd(self.motorIntakeSpeed)
                self.coralMotorR.setVelCmd(-1 * self.motorIntakeSpeed)
                pass
            elif self.coralCurState == CoralManState.EJECTING:
                self.coralMotorL.setVelCmd(self.motorEjectSpeed)
                self.coralMotorR.setVelCmd(-1 * self.motorEjectSpeed)
                pass
            elif self.coralCurState == CoralManState.HOLDING:
                self.coralMotorL.setVelCmd(self.motorHoldingSpeed)
                self.coralMotorR.setVelCmd(-1 * self.motorHoldingSpeed)
                pass

        pass

    def enableIntake(self):
        self.coralCurState = CoralManState.INTAKING
        pass

    def enableEject(self):
        self.coralCurState = CoralManState.EJECTING
        pass 
    
    def setDisabled(self):
        self.coralCurState = CoralManState.DISABLED

    def checkGamePiece(self):
        return False
    