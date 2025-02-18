from humanInterface.driverInterface import DriverInterface
from utils.singleton import Singleton
from ElevatorandMechConstants import CoralManState, AlgaeWristState, ElevatorLevelCmd
from ElevatorControl import ElevatorControl
from coralManipulatorControl import CoralManipulatorControl
from humanInterface.driverInterface import DriverInterface
from drivetrain.controlStrategies.autoDrive import AutoDrive
from humanInterface.operatorInterface import OperatorInterface

class MechSuperstructure(metaclass = Singleton): #Currently this isn't being called by anything 
    
    def __init__(self) -> None:
        self.targetLevel = ElevatorLevelCmd.L1
        self.targetPosition = 0 #should really be a position2D
        self.elev = ElevatorControl()
        self.dInt = DriverInterface()
        self.oInt = OperatorInterface()
        #self.drivetrainControl = DrivetrainControl() #This
        #self.poseEstimator = DrivetrainPoseEstimator(self.drivetrainControl.getModulePositions()) #This
        self.autoDrive = AutoDrive()
        self.currentlyAligning = False
        self.coralMan = CoralManipulatorControl() 
        self.ejectingGamepiece = False
        self.lHeights = [self.elev.L1_Height.get(),self.elev.L2_Height.get(),self.elev.L3_Height.get(),self.elev.L4_Height.get()]
        pass   
    
    def update(self):
        #self.poseEstimator.update() #This
        if self.dInt.getAutoDrive() and self.ejectingGamepiece == False: #I am currently assuming we want this to run alongside autodrive. If not I'll change it.
            self.currentlyAligning = True
        
        if self.currentlyAligning:
            if self.elev.getHeightM() - self.lHeights[self.elevatorTarget] <= 0.025 and self.autoDrive.getAtGoal(): #Checks to see if we are at the target height (Margin of error of 2 cm) and target pos
                self.coralMan.setCoralCmd(CoralManState.EJECTING) #If we are start the cycle for ejecting
                self.currentlyAligning = False 
                self.ejectingGamepiece = True 
        else:
            if self.ejectingGamepiece and self.coralMan.getCheckGamePiece() == False: #If we are ejecting and we don't have a game piece disable coralManipulator
                self.coralMan.setCoralCmd(CoralManState.DISABLED) 
                self.ejectingGamepiece = False
                self.currentlyAligning = False
            else:
                self.coralMan.setCoralCmd(self.oInt.getCoralCmd())
                self.coralMan.setAtL1(self.elev.getHeightM() < (self.elev.L1_Height.get() + 0.1))
                self.elev.setSafeToLeaveL1(self.coralMan.getCoralSafeToMove())
                self.elev.setManualAdjCmd(self.oInt.getElevManAdjCmd())
                self.elev.setHeightGoal(self.oInt.getElevCmd())
            
        pass 
    
    def activateSuperstructure(self, targetLevel, targetPos):
        self.elev.setHeightGoal(targetLevel)
        self.elevatorTarget = targetLevel
        self.targetPosition = targetPos
        self.currentlyAligning = True
        pass 

    def disableSuperstructure(self):
        self.elev.setHeightGoal(ElevatorLevelCmd.L1)
        self.currentlyAligning = False

    def modifyTargetLevel(self, targetLevel):
        self.elev.setHeightGoal(targetLevel)
        self.elevatorTarget = targetLevel
    
    def modifyTagetPosition(self, targetPos):
        self.targetPosition = targetPos