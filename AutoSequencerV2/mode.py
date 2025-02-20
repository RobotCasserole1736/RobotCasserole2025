from wpimath.geometry import Pose2d
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup


# An auto mode is something our robot might do during autonomous
# THe drive team selects the mode before the match
# The robot executes the mode during autonomous
# Modes must have a human-readable name, return a group of commands, and an
# initial drivetrain pose (IE, where is it expected the drive team placed the
# robot for this autonomous routine?)
class Mode:
    def __init__(self, name=None):
        if name:
            self._name = name
        else:
            self._name = self.__class__.__name__

    def getCmdGroup(self):
        return SequentialCommandGroup([])

    def getInitialDrivetrainPose(self) -> Pose2d | None:
        # Returns the initial pose of the robot for this particular auto mode
        # Or None, if the auto mode does not have a specific starting pose
        # If there is no specific starting pose, the only other source of 
        # starting pose is apriltags
        return None

    def getName(self):
        return self._name

    def __str__(self):
        return self.getName()