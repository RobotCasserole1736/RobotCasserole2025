from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from utils.allianceTransformUtils import transform
from utils.calibration import Calibration
from utils.constants import blueReefLocation
from utils.singleton import Singleton

class AutoSteer(metaclass=Singleton):
    def __init__(self):
        self.reefAlignActive = False
        self.returnDriveTrainCommand = DrivetrainCommand()
        self.rotKp = Calibration("Auto Align Rotation Kp",5)
        self.maxRotSpd = Calibration("Auto Align Max Rotate Speed", 6)

        # Previous Rotation Speed and time for calculating derivative
        self.prevDesAngle = 0
        self.prevTimeStamp = Timer.getFPGATimestamp()

        self.desiredAngle = 0

    def setReefAutoSteerCmd(self, shouldAutoAlign: bool):
        self.reefAlignActive = shouldAutoAlign
    
    def autoSteerIsRunning(self):
        return self.reefAlignActive
        

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:
        if self.reefAlignActive:
           # self.getDesiredSingerAngle(curPose)
            return self.calcReefDrivetrainCommand(curPose, cmdIn)
        else:
            return cmdIn

    def getRotationAngle(self, curPose: Pose2d) -> Rotation2d:
        targetLocation = transform(blueReefLocation)
        robotToTargetTrans = targetLocation - curPose.translation()
        return Rotation2d(robotToTargetTrans.X(), robotToTargetTrans.Y())

    def calcReefDrivetrainCommand(self, curPose: Pose2d, cmdIn: DrivetrainCommand) -> DrivetrainCommand:
        # Find difference between robot angle and angle facing the speaker
        rotError = self.getRotationAngle(curPose) - curPose.rotation()

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
    