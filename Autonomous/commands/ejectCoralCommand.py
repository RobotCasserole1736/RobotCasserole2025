
from wpilib import Timer
from AutoSequencerV2.sequentialCommandGroup import SequentialCommandGroup
from AutoSequencerV2.builtInCommands.waitCommand import WaitCommand
from AutoSequencerV2.command import Command
from Elevatorandmech.ElevatorandMechConstants import CoralManState
from Elevatorandmech.coralManipulatorControl import CoralManipulatorControl

class EjectCoralCommand(Command):
    def __init__(self, goingToL1=False):
        self.duration = .5
        self.atL1 = goingToL1
        self.hasPiece = True
        self.startTime = 0

    def initialize(self):
        CoralManipulatorControl().setAtL1(self.atL1)
        CoralManipulatorControl().setCoralCmd(CoralManState.EJECTING)
        self.hadPiece = False

    def execute(self):
        # Eject

        self.hasPiece = CoralManipulatorControl().getCheckGamePiece()
        if not self.hasPiece and not self.hadPiece:
            self.startTime = Timer.getFPGATimestamp()
            self.hadPiece = True

    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
        # TODO - should this be done right away once the coral is ejected? Even if the timeout hasn't expired?
       # return Timer.getFPGATimestamp() - self.startTime >= self.duration
        return not self.hasPiece and self.hadPiece and Timer.getFPGATimestamp() - self.startTime >= self.duration


    def end(self,interrupt):
        CoralManipulatorControl().setCoralCmd(CoralManState.DISABLED)
