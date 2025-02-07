
from AutoSequencerV2.command import Command
from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator

class EnableAprilTagCommand(Command):
    def __init__(self):
        self.drivetrainPoseEstimator = DrivetrainPoseEstimator(DrivetrainControl().getModulePositions())   

    def execute(self):
        self.drivetrainPoseEstimator.setUseAprilTags(True)

    def end(self, interrupted):
        self.drivetrainPoseEstimator.setUseAprilTags(True)

    def isDone(self):
        return False
        #return if we're done. Which should be never? It should be controlled by other things
