
from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.coralManipulatorControl import CoralManipulatorControl

class IntakeCoralCommand(Command):
    def __init__(self):
        self.duration = 3

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()

    def execute(self):
        # Intake
        CoralManipulatorControl().setCoralCommand(False,True,None)
        
    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
        return Timer.getFPGATimestamp() - self.startTime >= self.duration

    def end(self,interrupt):
        #set the inputs all to False, we don't want anything ejecting
        # coralcommandfile().update()
        CoralManipulatorControl().setCoralCommand(False,False,False)
        CoralManipulatorControl().update()