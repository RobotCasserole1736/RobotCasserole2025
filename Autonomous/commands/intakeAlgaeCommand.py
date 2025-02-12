
from wpilib import Timer
from AutoSequencerV2.command import Command
#from elevatorandmech.algaecontrol import AlgaeControlFile
from Elevatorandmech.algaeManipulatorControl import AlgeaIntakeControl

class IntakeAlgaeCommand(Command):
    def __init__(self):
        self.duration = 3

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()

    def execute(self):
        # Intake
        #algaecommandfile().setInput(everything, that, input, needs)
        AlgeaIntakeControl().setInput(True,False)
        
    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
       # return Timer.getFPGATimestamp() - self.startTime >= self.duration

        return AlgeaIntakeControl().checkHasGamePiece()

    def end(self,interrupt):
        AlgeaIntakeControl().setInput(False,False)
        AlgeaIntakeControl().update()