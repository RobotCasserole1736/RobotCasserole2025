from humanInterface.driverInterface import DriverInterface
from utils.singleton import Singleton
from ElevatorandMechConstants import CoralManState, AlgaeWristState, ElevatorLevelCmd
from ElevatorControl import ElevatorControl
from coralManipulatorControl import CoralManipulatorControl

class mechSuperstructure: #Currently this isn't being called by anything
    
    def __init__(self) -> None:
        self.targetLevel = ElevatorLevelCmd.L1
        self.targetPosition = 0 #should really be a position2D
        self.elevatorControl = ElevatorControl()
        self.currentlyAligning = False
        self.coralConMan = CoralManipulatorControl() #Should technically be called coralManCon for ManipulatorControl, but the idea of a coral con man is funnier.
        self.ejectingGamepiece = False
        self.lHeights = [self.elevatorControl.L1_Height.get(),self.elevatorControl.L2_Height.get(),self.elevatorControl.L3_Height.get(),self.elevatorControl.L4_Height.get()]
        pass   
    
    def update(self, curPos): #Ok so I'm pretty sure Robotpy teleopUpdate is gonna call this and so it will probably have the current position.

        if self.currentlyAligning:
            if self.elevatorControl.getHeightM() == self.lHeights[self.elevatorTarget] and curPos == self.targetPosition:
                self.coralConMan.setCoralCmd(CoralManState.EJECTING)
                self.currentlyAligning = False 
                self.ejectingGamepiece = True 
        if self.ejectingGamepiece and self.coralConMan.getCheckGamePiece(): 
                self.coralConMan.setCoralCmd(CoralManState.DISABLED)
                self.ejectingGamepiece = False
                self.currentlyAligning = False
        pass 
    
    def raiseToLevelAndScore(self, targetLevel, targetPos):
        self.elevatorControl.setHeightGoal(targetLevel)
        self.elevatorTarget = targetLevel
        self.targetPosition = targetPos 
        self.currentlyAligning = True
        pass 