
from wpilib import Timer
from AutoSequencerV2.command import Command
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.parallelCommandGroup import ParallelCommandGroup
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.elevatorHeightCommand import ElevatorHeightCommand
from Autonomous.commands.intakeCoralCommand import IntakeCoralCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from Autonomous.commands.ejectCoralCommand import EjectCoralCommand
from Autonomous.commands.intakeCoralCommandpt1 import IntakeCoralCommandPt1
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

class LCycleL4(Mode):
    def __init__(self):
        Mode.__init__(self, f"L Cycle L4")
        
        self.pathCmd1 = DrivePathCommand("LCycleL2P1")
        self.pathCmd2 = DrivePathCommand("LCycleL2P2")
        self.pathCmd3 = DrivePathCommand("LCycleL2P3")
        self.pathCmd4 = DrivePathCommand("LCycleL2P4")
        self.pathCmd5 = DrivePathCommand("LCycleL2P5")
        self.pathCmd6 = DrivePathCommand("LCycleL2P6")
        self.scoreL2 = EjectCoralCommand()
        self.intake = IntakeCoralCommand()
        self.startIntake = IntakeCoralCommandPt1()
        self.elev = ElevatorHeightCommand(ElevatorLevelCmd.L4)
        self.elevReturn = ElevatorHeightCommand(ElevatorLevelCmd.L1)

        self.step1Group = SequentialCommandGroup([self.pathCmd1, self.elev, self.scoreL2])
        self.path2Group = ParallelCommandGroup([self.elevReturn, self.pathCmd2])
        self.path3Group = ParallelCommandGroup([self.intake, self.pathCmd3])
        self.step4Group = SequentialCommandGroup([self.elev, self.scoreL2])
        self.path5Group = ParallelCommandGroup([self.elevReturn, self.pathCmd4])
        self.path6Group = ParallelCommandGroup([self.intake, self.pathCmd5])
        self.step7Group = SequentialCommandGroup([self.elev, self.scoreL2])
        self.path8Group = ParallelCommandGroup([self.elevReturn, self.pathCmd6])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.step1Group.andThen(self.path2Group).andThen(self.startIntake).andThen(self.path3Group).andThen(self.step4Group).andThen(
            self.path5Group).andThen(self.startIntake).andThen(self.path6Group).andThen(self.step7Group).andThen(self.path8Group).andThen(self.intake)

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(self.pathCmd1.path.get_initial_pose()))
