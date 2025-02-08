
from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.ElevatorandMechConstants import CoralManState
from Elevatorandmech.coralManipulatorControl import CoralManipulatorControl

class EjectCoralCommand(Command):
    def __init__(self, goingToL1=False):
        self.duration = 3
        self.atL1 = goingToL1

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        CoralManipulatorControl().setAtL1(self.atL1)

    def execute(self):
        # Eject
        CoralManipulatorControl().setCoralCmd(CoralManState.EJECTING)
        
    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
        # TODO - should this be done right away once the coral is ejected? Even if the timeout hasn't expired?
        return Timer.getFPGATimestamp() - self.startTime >= self.duration

    def end(self,interrupt):
        CoralManipulatorControl().setCoralCmd(CoralManState.DISABLED)
