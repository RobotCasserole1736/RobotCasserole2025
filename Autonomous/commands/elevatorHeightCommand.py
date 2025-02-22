from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.ElevatorControl import ElevatorControl
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from enum import Enum


class ElevatorHeightCommand(Command):

    def __init__(self, ElvLvlCmd):
        self.ElevLevel = ElvLvlCmd


    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
    
    def execute(self):

        ElevatorControl().setHeightGoal(self.ElevLevel)
    

    def isDone(self):
       # return Timer.getFPGATimestamp() - self.startTime >= self.duration
        return ElevatorControl().atHeight()
    
    def end(self,interrupt):
        ElevatorControl().setHeightGoal(ElevatorLevelCmd.NO_CMD)
        ElevatorControl().update()