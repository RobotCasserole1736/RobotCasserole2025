from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.raceCommandGroup import RaceCommandGroup
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.driveForwardSlowCommand import DriveForwardSlowCommand
from Autonomous.commands.ejectCoralCommand import EjectCoralCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from AutoSequencerV2.mode import Mode
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

# Just drives out of the starting zone. That's all.
class Center1CoralL1(Mode):
    def __init__(self):
        #this is naming the mode, in this case "Drive Out"
        Mode.__init__(self, f"Center 1 Coral L1")

        #This is setting the path command (pathCmd), which is what we will use. The DrivePathCommand must be 
        #exactly the same as it is in the Choreo name. 
        self.pathCmd = DrivePathCommand("Center 1 Coral L1")
        self.wait = WaitCommand(1.5)
        self.scoreL1 = EjectCoralCommand(True)
        self.driveSlow = DriveForwardSlowCommand()
        self.driveSlowGroup = RaceCommandGroup([self.wait, self.driveSlow])
        self.group = SequentialCommandGroup([self.pathCmd,self.wait,self.scoreL1])

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.pathCmd.andThen(self.driveSlowGroup).andThen(self.scoreL1)

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(self.pathCmd.path.get_initial_pose()))
