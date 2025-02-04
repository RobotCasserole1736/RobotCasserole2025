from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.ElevatorControl import ElevatorControl
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from enum import Enum


class ElevatorHeightCommand(Command):

    def __init__(self, ElvLvlCmd):
        self.ElevLevel = ElvLvlCmd
        self.duration = 3

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
    
    def execute(self):

        ElevatorControl().setHeightGoal(self.ElevLevel)
    
    def maxDuration(self,duration):
        self.duration = duration + 1

    def isDone(self):
        return Timer.getFPGATimestamp() - self.startTime >= self.duration

    def end(self,interrupt):
        ElevatorControl().setHeightGoal(ElevatorLevelCmd.NO_CMD)
        ElevatorControl().update()