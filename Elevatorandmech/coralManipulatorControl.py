from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd, CoralManState
from utils.calibration import Calibration
from utils.constants import CORAL_L_CANID, CORAL_R_CANID, CORAL_GAME_PIECE_B_PORT, CORAL_GAME_PIECE_F_PORT
from utils.signalLogging import addLog
from utils.singleton import Singleton
from wpilib import DigitalInput
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class CoralManipulatorControl(metaclass=Singleton):
    def __init__(self) -> None:
        self.coralCurState = CoralManState.DISABLED
        self.coralMotorL = WrapperedSparkMax(CORAL_L_CANID, "CoralMotorL", True, 10)
        self.coralMotorR = WrapperedSparkMax(CORAL_R_CANID, "CoralMotorR", True, 10)
        self.coralMotorR.setInverted(True)
        #we're assuming B is the one that hits first (it's closer to the intake side), while F is closer to front of eject side
        self.gamepieceSensorF = DigitalInput(CORAL_GAME_PIECE_F_PORT) 
        self.gamepieceSensorB = DigitalInput(CORAL_GAME_PIECE_B_PORT)   
        self.motorHoldingVoltage = Calibration("MotorHolding", 1.0, "V") 
        self.motorIntakeVoltage =  Calibration("MotorIntake", 8.0, "V")
        self.motorEjectVoltage =  Calibration("MotorEject", 11.0, "V") 
        self.RMotorEjectVoltageL1 = Calibration("MotorEjectRForL1", 5.0, "V")
        self.atL1 = False

        addLog("Coral Manipulator State", lambda:self.coralCurState.value, "state")
        addLog("Has Game Piece", self.getCheckGamePiece, "Bool")

    def update(self) -> None:
        if self.coralCurState == CoralManState.DISABLED:
            self.coralMotorL.setVoltage(0)
            self.coralMotorR.setVoltage(0)
        elif self.getCheckGamePiece():
            if (self.coralCurState == CoralManState.EJECTING):
                #for eject, we need to be able to set one of the motors (I chose right) to less if we're scoring L1
                self.coralMotorL.setVoltage(self.motorEjectVoltage.get())
                if self.atL1:
                    self.coralMotorR.setVoltage(self.RMotorEjectVoltageL1.get())
                else:
                    self.coralMotorR.setVoltage(self.motorEjectVoltage.get())
            elif self.coralCurState == CoralManState.HOLDING:
                self.coralMotorL.setVoltage(self.motorHoldingVoltage.get())
                self.coralMotorR.setVoltage(self.motorHoldingVoltage.get())
            # Disable by default
            else:
                self.coralMotorL.setVoltage(0)
                self.coralMotorR.setVoltage(0)
        else:
            if self.coralCurState == CoralManState.INTAKING:
                self.coralMotorL.setVoltage(self.motorIntakeVoltage.get())
                self.coralMotorR.setVoltage(self.motorIntakeVoltage.get())
            # Disable by default
            else:
                self.coralMotorL.setVoltage(0)
                self.coralMotorR.setVoltage(0)

    def getCheckGamePiece(self) -> bool:
        """We think the back sensor (the one the coral hits first) needs to be clear to have a game piece.
        And the front sensor needs to be tripped.
        For now, we want to assume we don't need to feed back.   """
        return self.gamepieceSensorF.get() and not self.gamepieceSensorB.get()

    def getCoralSafeToMove(self) -> bool:
        #I think this is just a function that is going to be used by elevator control. 
        # theoretically, As long as the back gamepiece sensor isn't being tripped, the robot is good to up because a coral isn't in the way. 
        return not self.gamepieceSensorB.get()

    def setCoralCommand(self, cmd: CoralManState, gotoL1) -> None:
        self.coralCurState = cmd
        self.atL1 = (gotoL1 == ElevatorLevelCmd.L1)
