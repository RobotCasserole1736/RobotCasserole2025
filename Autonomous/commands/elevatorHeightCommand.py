from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.ElevatorControl import ElevatorControl
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from enum import Enum


class ElevatorHeightCommand(Command):

    def __init__(self, ElvLvlCmd):
        self.ElevLevel = ElvLvlCmd
        self.duration = 4


    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        ElevatorControl().setHeightGoal(self.ElevLevel)

    
    def execute(self):
        pass
    

    def isDone(self):
        return ElevatorControl().atHeight() or Timer.getFPGATimestamp() - self.startTime >= self.duration
    
    def end(self,interrupt):
        ElevatorControl().setHeightGoal(ElevatorLevelCmd.NO_CMD)
        ElevatorControl().update()