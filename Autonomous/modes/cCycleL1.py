
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.intakeCoralCommand import IntakeCoralCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from Autonomous.commands.ejectCoralCommand import EjectCoralCommand
from utils.allianceTransformUtils import transform

class CCycleL1(Mode):
    def __init__(self):
        Mode.__init__(self, f"C Cycle L1")
        
        self.pathCmd1 = DrivePathCommand("CCycleL1P1")
        self.pathCmd2 = DrivePathCommand("CCycleL1P2")
        self.pathCmd3 = DrivePathCommand("CCycleL1P3")
        self.pathCmd4 = DrivePathCommand("CCycleL1P4")
        self.pathCmd5 = DrivePathCommand("CCycleL1P5")
        self.pathCmd6 = DrivePathCommand("CCycleL1P6")
        self.pathCmd7 = DrivePathCommand("CCycleL1P7")
        self.pathCmd8 = DrivePathCommand("CCycleL1P8")
        self.scoreL1 = EjectCoralCommand(True)
        self.intake = IntakeCoralCommand()
        self.group = SequentialCommandGroup([self.pathCmd1,self.scoreL1,self.pathCmd2,self.intake,self.pathCmd3,self.scoreL1,self.pathCmd4,self.intake,self.pathCmd5,self.scoreL1,self.pathCmd6,self.intake,self.pathCmd7,self.scoreL1,self.pathCmd8,self.intake])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.group

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return transform(self.pathCmd1.path.get_initial_pose())
