import wpilib
import wpimath 
from utils.singleton import Singleton
from enum import Enum
import utils.constants
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class AlgaeIntakeState(Enum):
   DISABLED = 0
   INTAKE = 1
   EJECT = 2
   HOLDING = 3

class AlgaeWristState(Enum):
    DISABLED = 4
    INTAKEOFFGROUND = 5
    STOW = 6
    REEF = 7

class AlgaeWristControl(metaclass=Singleton):

     def __init__(self):
        self.WristCurState = AlgaeWristState.DISABLED
        self.wristMotor = WrapperedSparkMax(16, "CoralWristMotor", True, 10)
        self.curMotorVoltage = 0.0
     def update(self):
        
        self.algaeCurState = AlgaeIntakeState.HOLDING
        
     
     def enableAlgaeIntakeoffGround(self):
         
         
class AlgeaIntakeControl(metaclass=Singleton):

    def __init__(self):
        self.algaeCurState = AlgaeIntakeState.DISABLED
        self.algaeMotor = WrapperedSparkMax(15, "CoralIntakeMotor", True, 10)
        self.gamepieceSensor = None # game piece sensor empty for now
        pass

    def update(self):
        
        self.hasGamePiece = self.checkGamePiece() #Check if we are holding a piece of coral. this way it will automaticly switch to holding mode
        if self.hasGamePiece:
            self.algaeCurState = AlgaeIntakeState.HOLDING

    def getAlgaeIntake(self):
        self.algaeCurState = AlgaeIntakeState.INTAKE
        pass
         

    def enableEject(self):
        self.algaeCurState = AlgaeIntakeState.EJECT
        pass 

    def disableIntake(self):
        if self.checkGamePiece():
            self.algaeCurState = AlgaeIntakeState.HOLDING
        else:
            self.algaeCurState.DISABLED

    def disableEject(self):
        pass 
    
    def checkGamePiece(self):
        if self.gamepieceSensor return true 
            
   