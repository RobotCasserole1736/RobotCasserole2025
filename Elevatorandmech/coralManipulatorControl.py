import wpilib
import wpimath 
from utils.calibration import Calibration
from utils.singleton import Singleton
from enum import Enum 
from wpilib import DigitalInput
from utils.constants import CORAL_L_CANID, CORAL_R_CANID, CORAL_GAME_PIECE_B_PORT, CORAL_GAME_PIECE_F_PORT
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from utils.signalLogging import addLog

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
        #we're assuming B is the one that hits first (it's closer to the intake side), while F is closer to front of eject side
        self.gamepieceSensorF = DigitalInput(CORAL_GAME_PIECE_F_PORT) 
        self.gamepieceSensorB = DigitalInput(CORAL_GAME_PIECE_B_PORT)   
        self.motorHoldingVoltage = Calibration("MotorHolding", 1.0, "V") 
        self.motorIntakeVoltage =  Calibration("MotorIntake", 8.0, "V")
        self.motorEjectVoltage =  Calibration("MotorEject", -11.0, "V") 
        self.RMotorEjectVoltageL1 = Calibration("MotorEjectRForL1", -5.0, "V")
        self.overrideToEject = False
        self.ejectCoral = False
        self.hasGamePiece = False
        self.atL1 = False

        addLog("Coral Enum", lambda:self.coralCurState.value, "state")
        addLog("Has Game Piece", lambda:self.hasGamePiece, "Bool")

    def update(self):
        self.hasGamePiece = self.getCheckGamePiece()

        #state to set what current state we're in
        #if there's an eject command, we just want to eject right away
        if self.ejectCoral and self.hasGamePiece:
            self.coralCurState = CoralManState.EJECTING
        #if we have a game piece, we hold it
        elif self.hasGamePiece:
            self.coralCurState = CoralManState.HOLDING
        #if we don't have a gamepiece and should be intaking, then intake
        elif self.intakeCoralAuto and not self.hasGamePiece:
            self.coralCurState = CoralManState.INTAKING
        #if nothing else, we stop
        else:
            self.coralCurState = CoralManState.DISABLED

        #set next state based on those
        if self.coralCurState == CoralManState.INTAKING:
            #If we have a gamepiece, the next state is holding
            #if not, we don't want to change the state - keep it at intaking
            if self.hasGamePiece:
                self.coralNextState = CoralManState.HOLDING
        #if we are ejecting coral
        elif self.coralCurState == CoralManState.EJECTING:
            #once we no longer have a game piece, we want to stop turning the motors
            #otherwise, just keep the state - don't set it to anything new. 
            if not self.hasGamePiece:
                self.coralNextState = CoralManState.DISABLED 
                             
        self.coralCurState = self.coralNextState
        
        #actually set the voltages
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
    
    def getCheckGamePiece(self):
        """We think the back sensor (the one the coral hits first) needs to be clear to have a game piece.
        And the front sensor needs to be tripped.
        For now, we want to assume we don't need to feed back.   """
        return self.gamepieceSensorF.get() and not self.gamepieceSensorB.get()
    
    def getcoralSafeToMove(self): 
        #I think this is just a function that is going to be used by elevator control. 
        # theoretically, As long as the back gamepiece sensor isn't being tripped, the robot is good to up because a coral isn't in the way. 
        return not self.gamepieceSensorB.get()
    
    def setCoralAutoIntake(self):
        self.intakeCoralAuto = True

    def setCoralCommand(self, Coraleject, autoIntake, gotoL1):
        #if we're at the ejecting, we want to run coral
        self.ejectCoral = Coraleject
        self.intakeCoralAuto = autoIntake
        self.atL1 = gotoL1