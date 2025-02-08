
from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.algaeManipulatorControl import AlgeaIntakeControl

class EjectAlgaeCommand(Command):
    def __init__(self):
        self.duration = 3

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()

    def execute(self):
        # Eject
        #algaecommandfile().setInput(everything, that, input, needs)
        AlgeaIntakeControl().setInput(False,True)
        
    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
        return Timer.getFPGATimestamp() - self.startTime >= self.duration

    def end(self,interrupted):
        #set the inputs all to False, we don't want anything ejecting
        AlgeaIntakeControl().setInput(False,False)
        AlgeaIntakeControl().update()