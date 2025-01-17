from Autonomous.commands.drivePathCommand import DrivePathCommand
from AutoSequencerV2.mode import Mode
from utils.allianceTransformUtils import transform

# Just drives out of the starting zone. That's all.
class driveTest1(Mode):
    def __init__(self):
        #this is naming the mode, in this case "Drive Out"
        Mode.__init__(self, f"drive test 1")

        #This is setting the path command (pathCmd), which is what we will use. The DrivePathCommand must be 
        #exactly the same as it is in the Choreo name. 
        self.pathCmd = DrivePathCommand("DriveTest1")

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.pathCmd

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return transform(self.pathCmd.path.get_initial_pose())
