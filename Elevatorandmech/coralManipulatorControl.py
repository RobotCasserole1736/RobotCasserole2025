from Elevatorandmech.ElevatorandMechConstants import CoralManState
from utils.calibration import Calibration
from utils.constants import CORAL_IN_CANID, CORAL_L_CANID, CORAL_R_CANID, CORAL_GAME_PIECE_B_PORT, CORAL_GAME_PIECE_F_PORT
from utils.singleton import Singleton
from utils.signalLogging import addLog
from wpilib import DigitalInput
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class CoralManipulatorControl(metaclass=Singleton):

    def __init__(self) -> None:
        self.coralCurState = CoralManState.DISABLED
        self.coralMotorIn = WrapperedSparkMax(CORAL_IN_CANID, "CoralMotorIn", True, 10)
        self.coralMotorL = WrapperedSparkMax(CORAL_L_CANID, "CoralMotorL", True, 10)
        self.coralMotorR = WrapperedSparkMax(CORAL_R_CANID, "CoralMotorR", True, 10)
        self.coralMotorR.setInverted(True)
        #we're assuming B is the one that hits first (it's closer to the intake side), while F is closer to front of eject side
        self.gamepieceSensorF = DigitalInput(CORAL_GAME_PIECE_F_PORT)
        self.gamepieceSensorB = DigitalInput(CORAL_GAME_PIECE_B_PORT)
        self.motorHoldingVoltage = Calibration("MotorHolding", 0.0, "V")
        self.motorIntakeFastVoltage =  Calibration("MotorIntakeFast", 10.0, "V")
        self.motorIntakeSlowVoltage = Calibration("MotorIntakeSlow", 3.0, "V")
        self.motorEjectVoltage =  Calibration("MotorEject", 12.0, "V")
        self.RMotorEjectVoltageL1 = Calibration("MotorEjectRForL1", 3.0, "V")
        self.LMotorEjectVoltageL1 = Calibration("MotorEjectLForL1", 9.0, "V")
        self.atL1 = False

        addLog("Coral Enum", lambda:self.coralCurState.value, "state")
        addLog("Has Game Piece", self.getCheckGamePiece, "Bool")

    def update(self) -> None:
        # Eject if we want to eject
        if self.coralCurState == CoralManState.EJECTING:
            EVoltage = self.motorEjectVoltage.get()
            self.coralMotorIn.setVoltage(-self.motorIntakeSlowVoltage.get())
            if self.atL1:
                self.coralMotorR.setVoltage(self.RMotorEjectVoltageL1.get())
                self.coralMotorL.setVoltage(self.LMotorEjectVoltageL1.get())
                
            else:
                self.coralMotorR.setVoltage(EVoltage)
                self.coralMotorL.setVoltage(EVoltage)

        #Next, if we have a game piece, we don't want to do anything
        elif self.getCheckGamePiece():
            self.coralCurState = CoralManState.HOLDING
            self.coralMotorL.setVoltage(self.motorHoldingVoltage.get())
            self.coralMotorR.setVoltage(self.motorHoldingVoltage.get())
            self.coralMotorIn.setVoltage(0)
        #if we are close to being in the right position but aren't for any reason, we want to intake slowly
        elif self._frontSeesCoral() and self._backSeesCoral():
            self.coralMotorL.setVoltage(self.motorIntakeSlowVoltage.get())
            self.coralMotorR.setVoltage(self.motorIntakeSlowVoltage.get())
            self.coralMotorIn.setVoltage(0)
        #if we are trying to intake and don't have a game piece on front and back, we want to intake at full speed
        elif self.coralCurState == CoralManState.INTAKING:
            self.coralMotorL.setVoltage(self.motorIntakeFastVoltage.get())
            self.coralMotorR.setVoltage(self.motorIntakeFastVoltage.get())
            self.coralMotorIn.setVoltage(self.motorIntakeSlowVoltage.get())
        else:
            # Disable otherwise
            self.coralMotorL.setVoltage(0)
            self.coralMotorR.setVoltage(0)
            self.coralMotorIn.setVoltage(0)

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

    def setCoralCmd(self, cmdStateIn: CoralManState, ejectCoral=False) -> None:
        #we need commands to tell us what the coral motors should be doing
        #Operator also has the ability to eject, in case of emergency 
        if ejectCoral:
            self.coralCurState = CoralManState.EJECTING
        else:
            self.coralCurState = cmdStateIn

    def setAtL1(self, isAtL1: bool) -> None:
        self.atL1 = isAtL1

    def hasCoralAnywhere(self) -> bool:
        return self._backSeesCoral() or self._frontSeesCoral()