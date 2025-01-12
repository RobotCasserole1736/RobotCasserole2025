import os
import choreo
import choreo.trajectory
from wpilib import Timer
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainControl import DrivetrainControl
from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform


class DrivePathCommand(Command):
    def __init__(self, pathFile):
        self.name = pathFile

        self.trajCtrl = Trajectory()

        # Get the internal path file
        absPath = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "deploy",
                "choreo",
                pathFile,
            )
        )

        self.path = choreo.load_swerve_trajectory(absPath)
        self.done = False
        self.startTime = (
            -1
        )
       
        # we'll populate these for real later, just declare they'll exist
        self.duration = self.path.get_total_time()
        self.drivetrain = DrivetrainControl()
        self.poseTelem = self.drivetrain.poseEst._telemetry

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.poseTelem.setChoreoTrajectory(self.path)

    def execute(self):
        curTime = Timer.getFPGATimestamp() - self.startTime
        curState = self.path.sample_at(curTime)

        if(curState is not None):
            curState = transform(curState)
            self.trajCtrl.setCmd(curState)
        else: 
            self.trajCtrl.setCmd(None)

        if curTime >= self.duration:
            self.trajCtrl.setCmd(None)
            self.poseTelem.setChoreoTrajectory(None)
            self.done = True

    def isDone(self):
        return self.done

    def end(self,interrupt):
        self.trajCtrl.setCmd(None)
        self.poseTelem.setChoreoTrajectory(None)

    def getName(self):
        return f"Drive Trajectory {self.name}"