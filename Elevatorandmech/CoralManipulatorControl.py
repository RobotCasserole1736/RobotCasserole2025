import wpilib
import wpimath 
from utils.calibration import Calibration
from utils.singleton import Singleton
from enum import Enum 
from wpilib import DigitalInput
from utils.constants import CORAL_L_CANID, CORAL_R_CANID, CORAL_GAME_PIECE_B_PORT, CORAL_GAME_PIECE_F_PORT
from wrappers.wrapperedSparkMax import WrapperedSparkMax

# Manual Eject > Logical Coral Disabled > Automatically Decide

class CoralManState(Enum):
    DISABLED = 0
    INTAKING = 1
    EJECTING = 2 
    HOLDING = 3

class CoralManipulatorControl(metaclass=Singleton):

    def __init__(self):
        self.coralCurState = CoralManState.DISABLED
        self.coralNextState = CoralManState.DISABLED
        self.coralMotorL = WrapperedSparkMax(CORAL_L_CANID, "CoralMotorL", True, 10)
        self.coralMotorR = WrapperedSparkMax(CORAL_R_CANID, "CoralMotorR", True, 10)
        self.coralMotorR.setInverted(True)
        self.gamepieceSensorF = DigitalInput(CORAL_GAME_PIECE_F_PORT) 
        self.gamepieceSensorB = DigitalInput(CORAL_GAME_PIECE_B_PORT)   
        self.motorHoldingVoltage = Calibration("MotorHolding", 1.0, "V") 
        self.motorIntakeVoltage =  Calibration("MotorIntake", 8.0, "V")
        self.motorEjectVoltage =  Calibration("MotorEject", -11.0, "V") 
        self.RMotorEjectVoltageL1 = Calibration("MotorEjectRForL1", -5.0, "V")
        self.overrideToEject = False
        self.runCoral = False
        self.hasGamePiece = False
        self.atL1 = False

    def update(self):
        self.hasGamePiece = self.checkGamePiece()

        #Big state machine for the state of the coral ejector :(
        if self.overrideToEject:
            self.coralNextState = CoralManState.EJECTING
        elif self.runCoral and self.hasGamePiece:
            self.coralNextState = CoralManState.EJECTING
        elif self.coralCurState == CoralManState.INTAKING:
            #If we have a gamepiece, the next state is holding
            if self.hasGamePiece:
                self.coralNextState = CoralManState.HOLDING
        elif self.coralCurState == CoralManState.EJECTING:
            if not self.hasGamePiece:
                self.coralNextState = CoralManState.DISABLED 
                             
        self.coralCurState = self.coralNextState
        
        if self.coralCurState == CoralManState.DISABLED:
            self.coralMotorL.setVoltage(0)
            self.coralMotorR.setVoltage(0)
        elif self.coralCurState == CoralManState.EJECTING:
            EVoltage = self.motorEjectVoltage.get()
            #for eject, we need to be able to set one of the motors (I chose right) to less if we're scoring L1
            self.coralMotorL.setVoltage(EVoltage)
            if self.atL1:
                self.coralMotorR.setVoltage(self.RMotorEjectVoltageL1.get())
            else:
                self.coralMotorR.setVoltage(EVoltage)
        elif self.coralCurState == CoralManState.HOLDING:
            HVoltage = self.motorHoldingVoltage.get()
            self.coralMotorL.setVoltage(HVoltage)
            self.coralMotorR.setVoltage(HVoltage)
        elif self.coralCurState == CoralManState.INTAKING:
            IVoltage = self.motorIntakeVoltage.get()
            self.coralMotorL.setVoltage(IVoltage)
            self.coralMotorR.setVoltage(IVoltage)
    
    def checkGamePiece(self):
        #TODO - add logic that we like. 
        """We think the back sensor (the one the coral hits first) needs to be clear or coral while intaking, 
        then slowly feed back until it hits it. So it'd be 1/2 or so inches off (think rings feedback from last year).
        The front one (the one that hits second) needs to be clear in order for us to say we successfully ejected."""
        return self.gamepieceSensorF.get() and self.gamepieceSensorB.get()
    
    def coralSafeToMove(self): 
        #TODO - I think this won't be a function. It might be combined with checkGamePiece and become a state-type thing that we run through. TBD
        return self.gamepieceSensorF.get() and self.gamepieceSensorB.get() or not self.gamepieceSensorF.get() and not self.gamepieceSensorB.get()
    
    def setL1(self, gotoL1):
        self.atL1 = gotoL1

    def setCoralCommand(self, atHeight, coralEjectOveride):
        #if we're at the right height, we want to run coral
        self.runCoral = atHeight
        if coralEjectOveride:
            #if we're ejecting, we want to run coral
            self.runCoral = True