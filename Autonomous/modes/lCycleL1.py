
from wpilib import Timer
from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.command import Command
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.intakeCoralCommand import IntakeCoralCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from Autonomous.commands.ejectCoralCommand import EjectCoralCommand
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

class LCycleL1(Mode):
    def __init__(self):
        Mode.__init__(self, f"L Cycle L1")
        
        self.pathCmd1 = DrivePathCommand("LCycleL1P1")
        self.pathCmd2 = DrivePathCommand("LCycleL1P2")
        self.pathCmd3 = DrivePathCommand("LCycleL1P3")
        self.pathCmd4 = DrivePathCommand("LCycleL1P4")
        self.pathCmd5 = DrivePathCommand("LCycleL1P5")
        self.scoreL1 = EjectCoralCommand(True)
        self.intake = IntakeCoralCommand()
        self.group = SequentialCommandGroup([self.pathCmd1,self.scoreL1, self.pathCmd2, self.intake,self.pathCmd3,
                                              self.scoreL1,self.pathCmd4, self.intake,self.pathCmd5,self.scoreL1])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.group

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(self.pathCmd1.path.get_initial_pose()))
