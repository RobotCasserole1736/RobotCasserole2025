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
        self.motorHoldingVoltage = Calibration("MotorHolding", 1.0, "V")
        self.motorIntakeVoltage =  Calibration("MotorIntake", 8.0, "V")
        self.motorEjectVoltage =  Calibration("MotorEject", 11.0, "V")
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
        elif self.getCheckGamePiece():
            if self.coralCurState == CoralManState.EJECTING:
                EVoltage = self.motorEjectVoltage.get()
                self.coralMotorL.setVoltage(EVoltage)
                if self.atL1:
                    self.coralMotorR.setVoltage(self.RMotorEjectVoltageL1.get())
                else:
                    self.coralMotorR.setVoltage(EVoltage)
            # Hold otherwise
            else:
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
        #Update 2/12 - they both return "True" when they're not sensing anything. So I'm going to flip values
        return not self.gamepieceSensorF.get() and self.gamepieceSensorB.get()

    def getCoralSafeToMove(self) -> bool:
        #I think this is just a function that is going to be used by elevator control.
        # theoretically, As long as the back gamepiece sensor isn't being tripped, the robot is good to up because a coral isn't in the way.
        #Update 2/12 - they both return "True" when they're not sensing anything. So I'm going to flip values
        return self.gamepieceSensorB.get()

    def setCoralCmd(self, cmdStateIn: CoralManState) -> None:
        #we need commands to tell us what the coral motors should be doing
        self.coralCurState = cmdStateIn

    def setAtL1(self, isAtL1: bool) -> None:
        self.atL1 = isAtL1
