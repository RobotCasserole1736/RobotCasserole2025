from wpimath.geometry import Pose2d
from wpilib import DriverStation
from AutoSequencerV2.modeList import ModeList
from AutoSequencerV2.builtInModes.doNothingMode import DoNothingMode
from AutoSequencerV2.builtInModes.waitMode import WaitMode
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from Autonomous.modes.driveForwardSlowly import DriveForwardSlowly
from Autonomous.modes.driveOut import DriveOut

from Autonomous.modes.driveTest1 import driveTest1
from utils.singleton import Singleton
from utils.allianceTransformUtils import onRed
from utils.autonomousTransformUtils import setFlip

class AutoSequencer(metaclass=Singleton):
    """Top-level implementation of the AutoSequencer"""

    def __init__(self):
        # Have different delay modes for delaying the start of autonomous
        self.delayModeList = ModeList("Delay")
        self.delayModeList.addMode(WaitMode(0.0))
        self.delayModeList.addMode(WaitMode(3.0))
        self.delayModeList.addMode(WaitMode(6.0))
        self.delayModeList.addMode(WaitMode(9.0))

        # Create a flip selector
        self.flipModeList = ModeList("Flip")
        self.flipModeList.addMode("Left")
        self.flipModeList.addMode("Right")

        # Create a list of every autonomous mode we want
        self.mainModeList = ModeList("Main")
        self.mainModeList.addMode(DoNothingMode())
        #right now, DriveOut is all commented out, so we don't need to add it to the list. 
        self.mainModeList.addMode(DriveOut())
        self.mainModeList.addMode(DriveForwardSlowly())
        self.mainModeList.addMode(driveTest1())
        self.topLevelCmdGroup = SequentialCommandGroup()
        self.startPose = Pose2d()

        # Alliance changes require us to re-plan autonomous
        # This variable is used to help track when alliance changes
        self._prevOnRed = onRed()

        self.updateMode(force=True)  # Ensure we load the auto sequencer at least once.

    # Returns true if the alliance has changed since the last call
    def _allianceChanged(self):
        curRed = onRed()
        retVal = curRed != self._prevOnRed
        self._prevOnRed = curRed
        return retVal

    def addMode(self, newMode):
        self.mainModeList.addMode(newMode)

    # Call this periodically while disabled to keep the dashboard updated
    # and, when things change, re-init modes
    def updateMode(self, force=False):
        mainChanged = self.mainModeList.updateMode()
        delayChanged = self.delayModeList.updateMode()
        laneChanged = self.flipModeList.updateMode()
        if mainChanged or delayChanged or laneChanged or force or self._allianceChanged():
            setFlip(self.flipModeList.getCurMode() == "Right")
            mainMode = self.mainModeList.getCurMode()
            mainMode.__init__()
            delayMode = self.delayModeList.getCurMode()
            self.topLevelCmdGroup = delayMode.getCmdGroup().andThen(
                mainMode.getCmdGroup()
            )
            self.startPose = mainMode.getInitialDrivetrainPose()
            print(
                f"[Auto] New Modes Selected: {DriverStation.getAlliance()} {delayMode.getName()}, {self.flipModeList.getCurMode()} {mainMode.getName()}"
            )

    # Call this once during autonmous init to init the current command sequence
    def initialize(self):
        self.updateMode() # Last-shot update before starting autonomous
        print("[Auto] Starting Sequencer")
        self.topLevelCmdGroup.initialize()

    def update(self):
        self.topLevelCmdGroup.execute()

    def end(self):
        self.topLevelCmdGroup.end(True)
        print("[Auto] Sequencer Stopped")

    def getMainModeList(self):
        return self.mainModeList.getNames()

    def getMainModeNTTableName(self):
        return self.mainModeList.getModeTopicBase()

    def getDelayModeList(self):
        return self.delayModeList.getNames()

    def getDelayModeNTTableName(self):
        return self.delayModeList.getModeTopicBase()

    def getFlipModeList(self):
        return self.flipModeList.getNames()

    def getFlipModeNTTableName(self):
        return self.flipModeList.getModeTopicBase()

    def getStartingPose(self) -> Pose2d | None:
        # Returns the initial pose of the auto routine, if it has a defined one.
        return self.startPose
