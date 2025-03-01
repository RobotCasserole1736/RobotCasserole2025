from Elevatorandmech.ElevatorandMechConstants import CoralManState
from utils.calibration import Calibration
from utils.constants import CORAL_L_CANID, CORAL_R_CANID, CORAL_GAME_PIECE_B_PORT, CORAL_GAME_PIECE_F_PORT
from utils.singleton import Singleton
from utils.signalLogging import addLog
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
        self.motorHoldingVoltage = Calibration("MotorHolding", 0.0, "V")
        self.motorIntakeVoltage =  Calibration("MotorIntake", 3.0, "V")
        self.motorEjectVoltage =  Calibration("MotorEject", 12.0, "V")
        self.RMotorEjectVoltageL1 = Calibration("MotorEjectRForL1", 5.0, "V")
        self.atL1 = False

        addLog("Coral Enum", lambda:self.coralCurState.value, "state")
        addLog("Has Game Piece", self.getCheckGamePiece, "Bool")

    def update(self) -> None:
        # Always disable if commanded to disable
        if self.coralCurState == CoralManState.DISABLED:
            self.coralMotorL.setVoltage(0)
            self.coralMotorR.setVoltage(0)
        # Eject or hold if it has a game piece
        elif self.coralCurState == CoralManState.EJECTING:
            EVoltage = self.motorEjectVoltage.get()
            self.coralMotorL.setVoltage(EVoltage)
            if self.atL1:
                self.coralMotorR.setVoltage(self.RMotorEjectVoltageL1.get())
            else:
                self.coralMotorR.setVoltage(EVoltage)        
        elif self.getCheckGamePiece():
            self.coralCurState = CoralManState.HOLDING
            self.coralMotorL.setVoltage(self.motorHoldingVoltage.get())
            self.coralMotorR.setVoltage(self.motorHoldingVoltage.get())
        # Can intake if there is no game piece
        else:
            if self.coralCurState == CoralManState.INTAKING:
                self.coralMotorL.setVoltage(self.motorIntakeVoltage.get())
                self.coralMotorR.setVoltage(self.motorIntakeVoltage.get())
            # Disable otherwise
            else:
                self.coralMotorL.setVoltage(0)
                self.coralMotorR.setVoltage(0)

    def getCheckGamePiece(self) -> bool:
        """We think the back sensor (the one the coral hits first) needs to be clear to have a game piece.
        And the front sensor needs to be tripped.
        For now, we want to assume we don't need to feed back.   """
        return self._frontSeesCoral() and not self._backSeesCoral()

    def getCoralSafeToMove(self) -> bool:
        # inhibit elevator motion until the back sensor sees no coral
        return not self._backSeesCoral()
    
    def _frontSeesCoral(self) -> bool:
        return not self.gamepieceSensorF.get() # True means no coral seen. False means sensor sees some coral.
    
    def _backSeesCoral(self) -> bool:
        return not self.gamepieceSensorB.get() # True means no coral seen. False means sensor sees some coral.

    def setCoralCmd(self, cmdStateIn: CoralManState) -> None:
        #we need commands to tell us what the coral motors should be doing
        self.coralCurState = cmdStateIn

    def setAtL1(self, isAtL1: bool) -> None:
        self.atL1 = isAtL1
