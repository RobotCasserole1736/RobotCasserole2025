from wpilib import Timer
from wpimath.geometry import Pose2d, Translation2d
from drivetrain.controlStrategies.holonomicDriveController import HolonomicDriveController
from drivetrain.drivetrainCommand import DrivetrainCommand
from navigation.obstacleDetector import ObstacleDetector
from utils.autonomousTransformUtils import flip
from utils.signalLogging import addLog
from utils.singleton import Singleton
from navigation.repulsorFieldPlanner import RepulsorFieldPlanner
from navigation.navConstants import goalListTot
from drivetrain.drivetrainPhysical import MAX_DT_LINEAR_SPEED_MPS
from utils.allianceTransformUtils import transform
import math

# Maximum speed that we'll attempt to path plan at. Needs to be at least 
# slightly less than the maximum physical speed, so the robot can "catch up" 
# if it gets off the planned path
MAX_PATHPLAN_SPEED_MPS = 0.75 * MAX_DT_LINEAR_SPEED_MPS

class AutoDrive(metaclass=Singleton):
    def __init__(self):
        self._autoDrive = False
        self.rfp = RepulsorFieldPlanner()
        self._trajCtrl = HolonomicDriveController("AutoDrive")
        self._telemTraj = []
        self._obsDet = ObstacleDetector()
        self._olCmd = DrivetrainCommand()
        self._prevCmd:DrivetrainCommand|None = None
        self._plannerDur:float = 0.0
        # This name might be a wee bit confusing. It just keeps track if we were in auto
        # targeting the speaker last refresh.
        self._autoPrevEnabled = False
        self.targetIndexNumber = 0
        self.stuckTracker = 0 
        self.prevPose = Pose2d()
        self.lenList = []
        self.goalListTotwTransform = []
        # Used by getDashTargetPositionIndex() to convert the target numbers from the 
        # python standard to the dashboard/JS standard
        self.dashboardConversionList = [9, 11, 6, 8, 3, 5, 0, 2, 15, 17, 12, 14]
        #^ Bottom is the side facing our driver station.
        addLog("AutoDrive Proc Time", lambda:(self._plannerDur * 1000.0), "ms")

    def getGoal(self) -> Pose2d | None:
        return self.rfp.goal

    def setRequest(self, autoDrive) -> None:
        self._autoPrevEnabled = self._autoDrive
        self._autoDrive = autoDrive

    def getDashTargetPositionIndex(self) -> int: #Only use this for the dashboard. This Automatically converts the python standard for goals to the JS standard. 
        return self.dashboardConversionList[self.targetIndexNumber] 
        
    def updateTelemetry(self) -> None:        
        self._telemTraj = self.rfp.getLookaheadTraj()

    def getWaypoints(self) -> list[Pose2d]:
        return self._telemTraj
    
    def getObstacles(self) -> list[Translation2d]:
        return self.rfp.getObstacleTransList()
    
    def isRunning(self)->bool:
        return self.rfp.goal != None

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:
        
        startTime = Timer.getFPGATimestamp()

        self.lenList.clear()

        self.goalListTotwTransform.clear()

        retCmd = cmdIn # default - no auto driving

        for obs in self._obsDet.getObstacles(curPose):
            self.rfp.addObstacleObservation(obs)

        self.rfp._decayObservations()

        # Handle command changes
        """
        #This is a version that checks rotation first, then goes to the nearest coral spot of the two. I don't like
        if(self._autoDrive):
            curRot = curPose.rotation()
            for goalOption in goalListAngle:
                self.possibleRotList.append(abs(curRot.degrees().__sub__(goalOption.degrees())))
            #bestGoal should return the best goal side. It should be an integer. 
            bestGoal = self.possibleRotList.index(min(self.possibleRotList))
            target = curPose.nearest(goalListTot[bestGoal])
            self.rfp.setGoal(flip(transform(target)))
        else:
            self.rfp.setGoal(None)
        """
        """
        
        #version 3 - just based on only distance. I kind of like this one
        if (self._autoDrive):
            for goal in goalListTot:
                self.goalListTotwTransform.append(flip(transform(goal)))
            pose = curPose.nearest(self.goalListTotwTransform)
            self.rfp.setGoal(pose)
        else:
            self.rfp.setGoal(flip(transform(None)))
        """
        #version 2 - this is based on distance, then rotation if the distances are too close
        #but it's not really working. 

        if(not self._autoDrive):
            # Driving not requested, set no goal
            self.rfp.setGoal(None)
        elif(self._autoDrive and not self._autoPrevEnabled):
            # First loop of auto drive, calc a new goal based on current position
            for goalOption in goalListTot:
                goalWTransform = flip(transform(goalOption.translation()))
                self.lenList.append(goalWTransform.distance(curPose.translation()))

            #find the nearest one
            primeTargetIndex = self.lenList.index(min(self.lenList))
            primeTarget = flip(transform(goalListTot[primeTargetIndex]))
            #pop the nearest in order to find the second nearest
            self.lenList.pop(primeTargetIndex)
            #second nearest
            secondTargetIndex = self.lenList.index(min(self.lenList))
            secondTarget = flip(transform(goalListTot[secondTargetIndex]))
            #if they're close enough, look at rotation 
            closeEnough = abs(secondTarget.translation().distance(curPose.translation()) \
                              - primeTarget.translation().distance(curPose.translation())) <= 1.0
            difAngle = abs(secondTarget.rotation().degrees() - primeTarget.rotation().degrees()) >= 10
            if (closeEnough and difAngle):
                #checking rotation
                #dif in degrees
                curRot = curPose.rotation().degrees()
                primeTargetDiff = abs(primeTarget.rotation().degrees() - curRot)
                secondTargetDiff = abs(secondTarget.rotation().degrees() - curRot)
                if primeTargetDiff <= secondTargetDiff:
                    target = primeTarget
                    self.targetIndexNumber = primeTargetIndex
                else:
                    target = secondTarget
                    self.targetIndexNumber = secondTargetIndex
            else:
                target = primeTarget
                self.targetIndexNumber = primeTargetIndex
            self.rfp.setGoal(target)
        """
        if self._autoDrive:
            target = flip(transform(goalListTot[10]))
            self.rfp.setGoal(target)
        else:
            self.rfp.setGoal(None)
        """

        # If being asked to auto-align, use the command from the dynamic path planner
        if(self._autoDrive):
            
            # Open Loop - Calculate the new desired pose and velocity to get there from the 
            # repulsor field path planner
            if(self._prevCmd is None):
                initCmd = DrivetrainCommand(0,0,0,curPose) # TODO - init this from current odometry vel
                self._olCmd = self.rfp.update(initCmd, MAX_PATHPLAN_SPEED_MPS*0.02)
            else:
                self._olCmd = self.rfp.update(self._prevCmd, MAX_PATHPLAN_SPEED_MPS*0.02)

            # Add closed loop - use the trajectory controller to add in additional 
            # velocity if we're currently far away from the desired pose
            retCmd = self._trajCtrl.update2(self._olCmd.velX, 
                                            self._olCmd.velY, 
                                            self._olCmd.velT, 
                                            self._olCmd.desPose, curPose)    
            self._prevCmd = retCmd
        else:
            self._prevCmd = None

        self._plannerDur = Timer.getFPGATimestamp() - startTime

        #Set our curPos as the new old pose
        self.prevPose = curPose


        return retCmd
    