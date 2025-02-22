
from AutoSequencerV2.mode import Mode
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.elevatorHeightCommand import ElevatorHeightCommand
from Autonomous.commands.intakeCoralCommand import IntakeCoralCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from Autonomous.commands.ejectCoralCommand import EjectCoralCommand
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

class CCycleL2(Mode):
    def __init__(self):
        Mode.__init__(self, f"C Cycle L2")
        
        self.pathCmd1 = DrivePathCommand("CCycleL2P1")
        self.pathCmd2 = DrivePathCommand("CCycleL2P2")
        self.pathCmd3 = DrivePathCommand("CCycleL2P3")
        self.pathCmd4 = DrivePathCommand("CCycleL2P4")
        self.pathCmd5 = DrivePathCommand("CCycleL2P5")
        self.pathCmd6 = DrivePathCommand("CCycleL2P6")
        self.pathCmd7 = DrivePathCommand("CCycleL2P7")
        self.pathCmd8 = DrivePathCommand("CCycleL2P8")
        self.scoreL2 = EjectCoralCommand()
        self.intake = IntakeCoralCommand()
        self.elev = ElevatorHeightCommand(ElevatorLevelCmd.L2)
        self.elevReturn = ElevatorHeightCommand(ElevatorLevelCmd.L1)
        self.group = SequentialCommandGroup([self.pathCmd1,self.elev,self.scoreL2,self.elevReturn,self.pathCmd2,self.intake,self.pathCmd3,self.elev,self.scoreL2,self.elevReturn,self.pathCmd4,self.intake,self.pathCmd5,self.elev,self.scoreL2,self.elevReturn,self.pathCmd6,self.intake,self.pathCmd7,self.elev,self.scoreL2,self.elevReturn,self.pathCmd8,self.intake])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.group

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(self.pathCmd1.path.get_initial_pose()))
