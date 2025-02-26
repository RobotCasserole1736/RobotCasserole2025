
from wpilib import Timer
from AutoSequencerV2.command import Command
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.parallelCommandGroup import ParallelCommandGroup
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.elevatorHeightCommand import ElevatorHeightCommand
from Autonomous.commands.intakeCoralCommand import IntakeCoralCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from Autonomous.commands.ejectCoralCommand import EjectCoralCommand
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

class LCycleL2(Mode):
    def __init__(self):
        Mode.__init__(self, f"L Cycle L2")
        
        self.pathCmd1 = DrivePathCommand("LCycleL2P1")
        self.pathCmd2 = DrivePathCommand("LCycleL2P2")
        self.pathCmd3 = DrivePathCommand("LCycleL2P3")
        self.pathCmd4 = DrivePathCommand("LCycleL2P4")
        self.pathCmd5 = DrivePathCommand("LCycleL2P5")
        self.pathCmd6 = DrivePathCommand("LCycleL2P6")
        self.scoreL2 = EjectCoralCommand()
        self.intake = IntakeCoralCommand()
        self.elev = ElevatorHeightCommand(ElevatorLevelCmd.L2)
        self.elevReturn = ElevatorHeightCommand(ElevatorLevelCmd.L1)
        self.step1Group = SequentialCommandGroup([self.pathCmd1, self.elev, self.scoreL2])
        self.path2Group = ParallelCommandGroup([self.elevReturn, self.pathCmd2])
        self.step3Group = SequentialCommandGroup([self.intake, self.pathCmd3, self.elev, self.scoreL2])
        self.path4Group = ParallelCommandGroup([self.elevReturn, self.pathCmd4])
        self.step5Group = SequentialCommandGroup([self.intake, self.pathCmd5, self.elev, self.scoreL2])
        self.path6Group = ParallelCommandGroup([self.elevReturn, self.pathCmd6])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.step1Group.andThen(self.path2Group).andThen(self.step3Group).andThen(self.path4Group).andThen(
            self.step5Group).andThen(self.path6Group).andThen(self.intake)

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(self.pathCmd1.path.get_initial_pose()))
