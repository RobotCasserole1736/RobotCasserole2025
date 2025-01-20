import wpilib
import wpimath 
from utils.singleton import Singleton
from enum import Enum 
from wpilib import DigitalInput
from utils.constants import CORAL_L_CANID, CORAL_R_CANID, CORAL_GAME_PIECE_B_PORT, CORAL_GAME_PIECE_F_PORT


 
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class CoralManState(Enum):
    DISABLED = 0
    INTAKING = 1
    EJECTING = 2 
    HOLDING = 3

class CoralManipulatorControl(metaclass=Singleton):

    def __init__(self):
        self.coralCurState = CoralManState.DISABLED
        self.coralMotorL = WrapperedSparkMax(CORAL_L_CANID, "CoralMotorL", True, 10)
        self.coralMotorR = WrapperedSparkMax(CORAL_R_CANID, "CoralMotorR", True, 10)
        self.gamepieceSensorF = DigitalInput(CORAL_GAME_PIECE_F_PORT) 
        self.gamepieceSensorB = DigitalInput(CORAL_GAME_PIECE_B_PORT)   
        self.hasGamePiece = False
        self.motorHoldingVoltage = 1.0 #Replace these with real numbers please :) 
        self.motorIntakeVoltage = 0.0
        self.motorEjectVoltage = 0.0

        pass 

    def update(self):
        
        self.hasGamePiece = self.checkGamePiece() #Check if we are holding a piece of coral. This way it will automatically switch to holding mode.
        
        if self.coralCurState != CoralManState.DISABLED: #Check and make sure we aren't in the "disabled" state. Only really matters in the beggining of the match.
            if self.hasGamePiece and self.coralCurState != CoralManState.EJECTING: #We don't want to hold on to the game peice while trying to eject.
                self.coralCurState = CoralManState.HOLDING 
            
            if self.coralCurState == CoralManState.INTAKING: #Do what we do depending on what mode we are in
                self.coralMotorL.setVoltage(self.motorIntakeVoltage) #set motor values
                self.coralMotorR.setVoltage(-1 * self.motorIntakeVoltage)
            elif self.coralCurState == CoralManState.EJECTING:
                self.coralMotorL.setVoltage(self.motorEjectVoltage) #set motor values
                self.coralMotorR.setVoltage(-1 * self.motorEjectVoltage)

                if self.hasGamePiece == False: #check and see if we no longer have a gamepeice. If we no longer have one then stop ejecting and assume that we want to intake.
                    self.coralCurState = CoralManState.INTAKING 
            elif self.coralCurState == CoralManState.HOLDING:
                self.coralMotorL.setVoltage(self.motorHoldingVoltage) #set motor values
                self.coralMotorR.setVoltage(-1 * self.motorHoldingVoltage)

            
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
        return self.gamepieceSensorF.get()
    
    def setEnabled(self):
        """Recommended over '.enableIntake' for leaving disabled mode because this checks if the robot is holding a gamepeice before assuming that we want to intake"""
        if  self.checkGamePiece():
            self.coralCurState = CoralManState.HOLDING 
        else:
            self.coralCurState = CoralManState.INTAKING