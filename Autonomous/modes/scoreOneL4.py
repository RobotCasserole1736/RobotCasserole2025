from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.commands.drivePathCommand import DrivePathCommand
from AutoSequencerV2.mode import Mode
from Autonomous.commands.ejectCoralCommand import EjectCoralCommand
from Autonomous.commands.elevatorHeightCommand import ElevatorHeightCommand
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

# Just drives out of the starting zone. That's all.
class ScoreOneL4(Mode):
    def __init__(self):
        #this is naming the mode, in this case "Drive Out"
        Mode.__init__(self, f"Score one L4")

        #This is setting the path command (pathCmd), which is what we will use. The DrivePathCommand must be 
        #exactly the same as it is in the Choreo name. 
        self.pathCmd = DrivePathCommand("scoreOneL4")
        self.elev = ElevatorHeightCommand(ElevatorLevelCmd.L4)
        self.eject = EjectCoralCommand()
        self.elevReturn = ElevatorHeightCommand(ElevatorLevelCmd.L1)
        self.cmdGroup = SequentialCommandGroup([self.pathCmd, self.elev, self.eject, self.elevReturn ])
    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.cmdGroup

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return flip(transform(self.pathCmd.path.get_initial_pose()))