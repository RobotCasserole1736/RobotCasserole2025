from humanInterface.driverInterface import DriverInterface
from utils.singleton import Singleton
from ElevatorandMechConstants import CoralManState, AlgaeWristState, ElevatorLevelCmd
from ElevatorControl import ElevatorControl
from coralManipulatorControl import CoralManipulatorControl
from humanInterface.driverInterface import DriverInterface
from drivetrain.controlStrategies.autoDrive import AutoDrive
#from drivetrain.drivetrainControl import DrivetrainControl #This !
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator #THIS!

class mechSuperstructure(metaclass = Singleton): #Currently this isn't being called by anything 
    
    def __init__(self) -> None:
        self.targetLevel = ElevatorLevelCmd.L1
        self.targetPosition = 0 #should really be a position2D
        self.elevatorControl = ElevatorControl()
        self.driverInterface = DriverInterface()
        #self.drivetrainControl = DrivetrainControl() #This
        #self.poseEstimator = DrivetrainPoseEstimator(self.drivetrainControl.getModulePositions()) #This
        self.autoDrive = AutoDrive()
        self.currentlyAligning = False
        self.coralConMan = CoralManipulatorControl() 
        self.ejectingGamepiece = False
        self.lHeights = [self.elevatorControl.L1_Height.get(),self.elevatorControl.L2_Height.get(),self.elevatorControl.L3_Height.get(),self.elevatorControl.L4_Height.get()]
        pass   
    
    def update(self, curPos): # TODO - Check and see if it is worth recalculating all of the pos estimator stuff or if I can just get it from elsewhere.
        #self.poseEstimator.update() #This
        if self.driverInterface.getAutoDrive() and self.ejectingGamepiece == False: #I am currently assuming we want this to run alongside autodrive. If not I'll change it.
            self.currentlyAligning = True

        if self.currentlyAligning:
            if self.elevatorControl.getHeightM() - self.lHeights[self.elevatorTarget] <= 0.025 and self.autoDrive.getAtGoal(curPos): #Checks to see if we are at the target height (Margin of error of 2 cm) and target pos
                self.coralConMan.setCoralCmd(CoralManState.EJECTING) #If we are start the cycle for ejecting
                self.currentlyAligning = False 
                self.ejectingGamepiece = True 
        if self.ejectingGamepiece and self.coralConMan.getCheckGamePiece() == False: #If we are ejecting and we don't have a game piece disable coralManipulator
                self.coralConMan.setCoralCmd(CoralManState.DISABLED) 
                self.ejectingGamepiece = False
                self.currentlyAligning = False
        pass 
    
    def activateSuperstructure(self, targetLevel, targetPos):
        self.elevatorControl.setHeightGoal(targetLevel)
        self.elevatorTarget = targetLevel
        self.targetPosition = targetPos
        self.currentlyAligning = True
        pass 

    def disableSuperstructure(self):
        self.elevatorControl.setHeightGoal(ElevatorLevelCmd.L1)
        self.currentlyAligning = False

    def modifyTargetLevel(self, targetLevel):
        self.elevatorControl.setHeightGoal(targetLevel)
        self.elevatorTarget = targetLevel
    
    def modifyTagetPosition(self, targetPos):
        self.targetPosition = targetPos