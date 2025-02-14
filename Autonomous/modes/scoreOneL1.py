
from wpilib import Timer
from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.command import Command
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.elevatorHeightCommand import ElevatorHeightCommand
from Autonomous.commands.intakeCoralCommand import IntakeCoralCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from Autonomous.commands.ejectCoralCommand import EjectCoralCommand
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from utils.allianceTransformUtils import transform

class scoreOneL1(Mode):
    def __init__(self):
        Mode.__init__(self, f"Score One L1")
        
        self.pathCmd = DrivePathCommand("ScoreOne")
        self.wait = WaitCommand(0.2)
        self.eject = EjectCoralCommand()
        self.elev = ElevatorHeightCommand(ElevatorLevelCmd.L1)
        self.group = SequentialCommandGroup([self.pathCmd, self.elev, self.eject])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.group

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return transform(self.pathCmd.path.get_initial_pose())
