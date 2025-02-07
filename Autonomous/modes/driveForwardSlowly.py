from Autonomous.commands.driveForwardSlowCommand import DriveForwardSlowCommand
from Autonomous.commands.drivePathCommand import DrivePathCommand
from AutoSequencerV2.mode import Mode
from utils.allianceTransformUtils import transform

# Just drives out of the starting zone. That's all.
class DriveForwardSlowly(Mode):
    def __init__(self):
        Mode.__init__(self, f"Crawl")

        #This is setting the path command (pathCmd), which is what we will use. The DrivePathCommand must be 
        #exactly the same as it is in the Choreo name. 
        self.cmds = DriveForwardSlowCommand()

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.cmds

    def getInitialDrivetrainPose(self):
        # This auto routine can be started from anywhere
        # We rely on apriltags to make sure our pose is right
        # The auto routine will not reset pose at the start of match
        return None
