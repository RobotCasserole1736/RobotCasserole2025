from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from utils.allianceTransformUtils import transform
from utils.calibration import Calibration
from utils.constants import FIELD_Y_M, blueReefLocation
from utils.singleton import Singleton
from wpimath.filter import Debouncer

HP_STATION_HYSTERISIS_M = 2.0
HP_STATION_ANGLE_MAG_DEG = 54.0

class AutoSteer(metaclass=Singleton):
    def __init__(self):
        self.reefAlignActive = False
        self.returnDriveTrainCommand = DrivetrainCommand()
        self.rotKp = Calibration("Auto Align Rotation Kp",5)
        self.maxRotSpd = Calibration("Auto Align Max Rotate Speed", 6)
        self.hasCoralDbnc = True
        self.hasCoralDebouncer = Debouncer(0.5, Debouncer.DebounceType.kBoth)

        self.curTargetRot = Rotation2d()

    def setReefAutoSteerCmd(self, shouldAutoAlign: bool):
        self.reefAlignActive = shouldAutoAlign

    def setHasCoral(self, hasCoral: bool):
        self.hasCoralDbnc = self.hasCoralDebouncer.calculate(hasCoral)
        
    def autoSteerIsRunning(self):
        return self.reefAlignActive
        

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:
        if self.reefAlignActive:
            return self._calcAutoSteerDrivetrainCommand(curPose, cmdIn)
        else:
            return cmdIn

    def updateRotationAngle(self, curPose: Pose2d) -> None:

        if(self.hasCoralDbnc):
            targetLocation = transform(blueReefLocation)
            robotToTargetTrans = targetLocation - curPose.translation()
            self.curTargetRot = Rotation2d(robotToTargetTrans.X(), robotToTargetTrans.Y())
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
    