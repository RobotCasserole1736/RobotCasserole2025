
from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.coralManipulatorControl import CoralManipulatorControl

class EjectCoralCommand(Command):
    def __init__(self):
        self.duration = 3

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()

    def execute(self):
        # Eject
        #coralcommandfile().setInput(everything, that, input, needs)
        CoralManipulatorControl().setCoralCommand(True,False,None)
        
    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
        return Timer.getFPGATimestamp() - self.startTime >= self.duration

    def end(self,interrupt):
        CoralManipulatorControl().setCoralCommand(False,False,None)
        CoralManipulatorControl().update()