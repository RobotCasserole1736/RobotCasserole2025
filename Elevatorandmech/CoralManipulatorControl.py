import wpilib
import wpimath 
from utils.singleton import Singleton
from enum import Enum 
from wpilib import DigitalInput
from utils.constants import CORAL_L_CANID, CORAL_R_CANID, CORAL_GAME_PIECE_B_PORT, CORAL_GAME_PIECE_F_PORT
from humanInterface.driverInterface import DriverInterface


# Manual Eject > Logical Coral Disabled > Automatically Decide
 
from wrappers.wrapperedSparkMax import WrapperedSparkMax

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
        self.gamepieceSensorF = DigitalInput(CORAL_GAME_PIECE_F_PORT) 
        self.gamepieceSensorB = DigitalInput(CORAL_GAME_PIECE_B_PORT)   
        self.hasGamePiece = False
        self.motorHoldingVoltage = 1.0 #Replace these with real numbers please :) 
        self.motorIntakeVoltage = 0.0
        self.motorEjectVoltage = 0.0 # CC is Coral Cycle 
        self.overrideCCToEnabled = False #In my hopes and dreams this is only enabled when the driver presses the top POV 
        self.overrideCCToDisabled = True #Similarly, this would be enabled only while the bottom POV is pressed. We can replace these two with driverInterface/Auto commands later.
        self.overrideToEject = False
        self.coralCycleEnabled = False
        self.driverInterface = DriverInterface()
        pass 

    def update(self):
        
        if self.driverInterface.getEnableCoralCommand() == True:
            self.coralCycleEnabled = True
        elif self.driverInterface.getDisableCoralCommand() == True:
            self.coralCycleEnabled = False

        self.overrideToEject = self.driverInterface.getCoralEjectOverride()

        if not self.coralCycleEnabled: #Mini state machine for if the Coral Cycle is enabled :)
            if self.overrideCCToEnabled:
                self.coralCycleEnabled = True 
        else:
            if self.overrideCCToDisabled:
                self.coralCycleEnabled = False
        
        if self.coralCurState == CoralManState.DISABLED: #Big state machine for the state of the coral ejector :(
            if self.overrideToEject:
                self.coralNextState = CoralManState.EJECTING
            elif self.coralCycleEnabled:
                self.coralNextState = CoralManState.INTAKING 
        elif self.coralCurState == CoralManState.INTAKING:
            if self.overrideToEject:
                self.coralNextState = CoralManState.EJECTING
            else:
                if not self.coralCycleEnabled:
                    self.coralNextState = CoralManState.DISABLED
                else:    #If cycle is enabled
                    if self.checkGamePiece():
                        self.coralNextState = CoralManState.HOLDING
        elif self.coralCurState == CoralManState.HOLDING:  
            if self.overrideToEject:
                self.coralNextState = CoralManState.EJECTING 
            else:
                if not self.coralCycleEnabled:
                    self.coralNextState = CoralManState.DISABLED
                else:
                    if not self.checkGamePiece():
                        self.coralNextState = CoralManState.INTAKING
        elif self.coralCurState == CoralManState.EJECTING: 
            if not self.overrideToEject:
                if not self.coralCycleEnabled:
                    self.coralNextState = CoralManState.DISABLED
                elif self.gamepieceSensorB == False and self.gamepieceSensorF == False:
                    self.coralNextState = CoralManState.INTAKING
                             
        self.coralCurState = self.coralNextState
        
        if self.coralCurState == CoralManState.DISABLED:
            self.coralMotorL.setVoltage(0)
            self.coralMotorR.setVoltage(0)
        elif self.coralCurState == CoralManState.EJECTING:
            self.coralMotorL.setVoltage(self.motorEjectVoltage)
            self.coralMotorR.setVoltage(self.motorEjectVoltage * -1)
        elif self.coralCurState == CoralManState.HOLDING:
            self.coralMotorL.setVoltage(self.motorHoldingVoltage)
            self.coralMotorR.setVoltage(self.motorHoldingVoltage * -1)
        elif self.coralCurState == CoralManState.INTAKING:
            self.coralMotorL.setVoltage(self.motorIntakeVoltage)
            self.coralMotorR.setVoltage(self.motorIntakeVoltage * -1)

        pass

    def enableEject(self):
        self.overrideToEject = True
        pass

    def disableEject(self):
        self.overrideToEject = False

    def setDisabled(self):
        self.coralCycleEnabled = False
        pass 
    
    def setEnabled(self):
        """Recommended over '.enableIntake' for leaving disabled mode because this checks if the robot is holding a gamepeice before assuming that we want to intake"""
        self.coralCycleEnabled = True
    
    def checkGamePiece(self):
        return self.gamepieceSensorF.get() and self.gamepieceSensorB.get()