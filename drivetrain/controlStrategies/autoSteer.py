import math
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from navigation.autoSteerNavConstants import getTransformedGoalList
from utils.allianceTransformUtils import transform
from utils.calibration import Calibration
from utils.constants import FIELD_Y_M, blueReefLocation
from utils.singleton import Singleton
from wpimath.filter import Debouncer

from utils.units import deg2Rad

HP_STATION_HYSTERISIS_M = 1.0
HP_STATION_ANGLE_MAG_DEG = 54.0

ACTIVE_CMD_THRESH_TRANS_MPS = 0.1
ACTIVE_CMD_THRESH_ROT_RADPERS = deg2Rad(10)

class AutoSteer(metaclass=Singleton):
    def __init__(self):
        self.isActiveCmd = False
        self.alignToProcessor = False
        self.returnDriveTrainCommand = DrivetrainCommand()
        self.rotKp = Calibration("Auto Align Rotation Kp",5)
        self.maxRotSpd = Calibration("Auto Align Max Rotate Speed", 6)
        self.hasCoralDbncd = True
        self.hasCoralDebouncer = Debouncer(0.5, Debouncer.DebounceType.kBoth)
        
        self.hasIncomingRotCmdDbncd = False
        self.hasIncomingTransCmdDbncd = False

        self.incomingRotDebouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        self.incomingTransDebouncer = Debouncer(0.5, Debouncer.DebounceType.kBoth)

        self.manualCmdInhibit = False

        self.lenList= []

        self.isActive = False

        self.curGoalPose = None


        self.curTargetRot = Rotation2d()

    def setInhibited(self):
        # If we want to flip states one-shot back to inactive, call this
        # Should happen at teleop init to prevent motion without driver command
        self.manualCmdInhibit = True

    def _updateManualCmdInhibit(self, curCmd:DrivetrainCommand)->None:
        incomingRotCmdRaw = abs(curCmd.velT) > ACTIVE_CMD_THRESH_ROT_RADPERS
        incomingTransCmdRaw = abs(math.hypot(curCmd.velX, curCmd.velY)) > ACTIVE_CMD_THRESH_TRANS_MPS
        self.hasIncomingRotCmdDbncd = self.incomingRotDebouncer.calculate(incomingRotCmdRaw)
        self.hasIncomingTransCmdDbncd = self.incomingTransDebouncer.calculate(incomingTransCmdRaw)

        if(self.hasIncomingRotCmdDbncd):
            #any rotation command wins
            self.manualCmdInhibit = True
        elif(self.hasIncomingTransCmdDbncd):
            #only go back once we're translating
            self.manualCmdInhibit = False
        else:
            # Maintain old state
            pass

    def setAutoSteerActiveCmd(self, shouldAutoAlign: bool):
        self.isActiveCmd = shouldAutoAlign

    def setAlignToProcessor(self, alignToProcessor: bool):
        self.alignToProcessor = alignToProcessor

    def setAlignDownfield(self, alignDownField: bool):
        self.alignDownfield = alignDownField

    def setHasCoral(self, hasCoral: bool):
        self.hasCoralDbncd = self.hasCoralDebouncer.calculate(hasCoral)
        
    def isRunning(self):
        return self.isActive

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:

        self._updateManualCmdInhibit(cmdIn)

        if self.isActiveCmd and not self.manualCmdInhibit:
            self.isActive = True
            return self._calcAutoSteerDrivetrainCommand(curPose, cmdIn)
        else:
            self.isActive = False
            return cmdIn

    def getCurGoalPose(self) -> Pose2d|None:
        return self.curGoalPose
    
    def updateRotationAngle(self, curPose: Pose2d) -> None:

        self.lenList.clear()
        self.curGoalPose = None

        if(self.alignToProcessor):
            self.curTargetRot = transform(Rotation2d.fromDegrees(-90.0))
        elif(self.alignDownfield):
            self.curTargetRot = transform(Rotation2d.fromDegrees(0.0))
        elif(self.hasCoralDbncd):
            goalListTot = getTransformedGoalList()

            for goalOption in goalListTot:
                goalWTransform = goalOption.translation()
                self.lenList.append(goalWTransform.distance(curPose.translation()))
            primeTargetIndex = self.lenList.index(min(self.lenList))
            targetLocation = goalListTot[primeTargetIndex].translation()

            robotToTargetTrans = targetLocation - curPose.translation()
            self.curTargetRot = Rotation2d(robotToTargetTrans.X(), robotToTargetTrans.Y())
            self.curGoalPose = Pose2d(targetLocation, Rotation2d())
        else:
            # Check if we are closer to the left or right human player station
            curY = transform(curPose.translation()).Y()
            if(curY > (FIELD_Y_M / 2.0) +  HP_STATION_HYSTERISIS_M/2.0):
                # Closer to left human player station
                self.curTargetRot  = transform(Rotation2d.fromDegrees(-HP_STATION_ANGLE_MAG_DEG))
            elif(curY < (FIELD_Y_M / 2.0) - HP_STATION_HYSTERISIS_M/2.0):
                # Closer to right human player station
                self.curTargetRot  = transform(Rotation2d.fromDegrees(HP_STATION_ANGLE_MAG_DEG))
            else:
                # Within hysterisis band, keep command unchanged
                pass


    def _calcAutoSteerDrivetrainCommand(self, curPose: Pose2d, cmdIn: DrivetrainCommand) -> DrivetrainCommand:

        # Update our target location
        self.updateRotationAngle(curPose)

        # Find difference between robot angle and angle facing the speaker
        rotError = self.curTargetRot - curPose.rotation()

        # Check to see if we are making a really small correction
        # If we are, don't worry about it. We only need a certain level of accuracy
        if abs(rotError.radians()) <= 0.05:
            rotError = 0
        else:
            rotError = rotError.radians()

        self.returnDriveTrainCommand.velT = min(rotError*self.rotKp.get(),self.maxRotSpd.get())
        self.returnDriveTrainCommand.velX = cmdIn.velX # Set the X vel to the original X vel
        self.returnDriveTrainCommand.velY = cmdIn.velY # Set the Y vel to the original Y vel
        return self.returnDriveTrainCommand
    