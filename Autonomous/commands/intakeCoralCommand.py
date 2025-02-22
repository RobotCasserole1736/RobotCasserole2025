
from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.ElevatorandMechConstants import CoralManState
from Elevatorandmech.coralManipulatorControl import CoralManipulatorControl

class IntakeCoralCommand(Command):
    def __init__(self):
        self.duration = 3

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()

    def execute(self):
        # Intake
        CoralManipulatorControl().setCoralCmd(CoralManState.INTAKING)
        
    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
        #return Timer.getFPGATimestamp() - self.startTime >= self.duration
        return CoralManipulatorControl().getCheckGamePiece()


    def end(self,interrupt):
        CoralManipulatorControl().setCoralCmd(CoralManState.DISABLED)
