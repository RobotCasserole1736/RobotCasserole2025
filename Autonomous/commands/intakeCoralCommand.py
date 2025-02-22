
from wpilib import Timer
from AutoSequencerV2.command import Command
from Elevatorandmech.ElevatorandMechConstants import CoralManState, ElevatorLevelCmd
from Elevatorandmech.coralManipulatorControl import CoralManipulatorControl
from Elevatorandmech.ElevatorControl import ElevatorControl

class IntakeCoralCommand(Command):
    def __init__(self):
        self.duration = 3

    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()

    def execute(self):
        # Intake
        ElevatorControl().setHeightGoal(ElevatorLevelCmd.L1)
        CoralManipulatorControl().setCoralCmd(CoralManState.INTAKING)

    def maxDuration(self, duration):
        self.duration = duration + 1

    def isDone(self):
        return (Timer.getFPGATimestamp() - self.startTime >= self.duration)
            or (CoralManipulatorControl().getCheckGamePiece())

    def end(self,interrupt):
        ElevatorControl().setHeightGoal(ElevatorLevelCmd.NO_CMD)
        CoralManipulatorControl().setCoralCmd(CoralManState.DISABLED)
