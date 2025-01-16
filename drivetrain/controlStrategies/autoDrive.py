from wpilib import Timer
from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import Trajectory
from drivetrain.controlStrategies.holonomicDriveController import HolonomicDriveController
from drivetrain.drivetrainCommand import DrivetrainCommand
from navigation.obstacleDetector import ObstacleDetector
from utils.signalLogging import addLog
from utils.singleton import Singleton
from navigation.repulsorFieldPlanner import RepulsorFieldPlanner
from navigation.navConstants import GOAL_PICKUP, GOAL_SPEAKER, goalListAngle, goalListTot
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
        self.autoPrevEnabled = False #This name might be a wee bit confusing. It just keeps track if we were in auto targeting the speaker last refresh.
        self.stuckTracker = 0 
        self.prevPose = Pose2d()
        self.LenList = []

        addLog("AutoDrive Proc Time", lambda:(self._plannerDur * 1000.0), "ms")


    def setRequest(self, autoDrive) -> None:
        self._autoDrive = autoDrive
        #The following if statement is just logic to enable self.autoPrevEnabled when the driver enables an auto.
        if self.autoPrevEnabled != self._autoDrive:
            self.stuckTracker = 0
        self.autoPrevEnabled = self._autoDrive
        
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

        self.LenList.clear()

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
            self.rfp.setGoal(transform(target))
        else:
            self.rfp.setGoal(None)
        """
        """
        #version 3 - just based on only distance. I kind of like this one
        if (self._autoDrive):
            pose = curPose.nearest(goalListTot)
            self.rfp.setGoal(pose)
        else:
            self.rfp.setGoal(transform(None))
        """

        #version 2 - this is based on distance, then rotation if the distances are too close
        if (self._autoDrive):
            for goalOption in goalListTot:
                self.LenList.append(goalOption.translation().distance(curPose.translation()))

            #find the nearest one
            primeTargetIndex = self.LenList.index(min(self.LenList))
            primeTarget = goalListTot[primeTargetIndex]
            #pop the nearest in order to find the second nearest
            self.LenList.pop(primeTargetIndex)
            #second nearest
            secondTargetIndex = self.LenList.index(min(self.LenList))
            secondTarget = goalListTot[secondTargetIndex]
            #if they're close enough, look at rotation 
            if (abs(secondTarget.translation().distance(curPose.translation()) - primeTarget.translation().distance(curPose.translation())) <= 1.0):
                #checking rotation
                #dif in degrees
                curRot = curPose.rotation().degrees()
                primeTargetDiff = abs(primeTarget.rotation().degrees() - curRot)
                secondTargetDiff = abs(secondTarget.rotation().degrees() - curRot)
                if primeTargetDiff <= secondTargetDiff:
                    target = primeTarget
                else:
                    target = secondTarget
            else:
                target = primeTarget
            self.rfp.setGoal(target)
        else:
            self.rfp.setGoal(None)


        # If being asked to auto-align, use the command from the dynamic path planner
        if(self._autoDrive):
            if self.stuckTracker < 10: #Only run if the robot isn't stuck
                #This checks how much we moved, and if we moved less than one cm it increments the counter by one.
                if math.sqrt(((curPose.X() - self.prevPose.X()) ** 2) + ((curPose.Y() - self.prevPose.Y()) ** 2)) < .01:
                    self.stuckTracker += 1
                else:
                    if self.stuckTracker > 0:
                        self.stuckTracker -= 1

            
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
        #assume that we are either stuck or done if the counter reaches above 15. (sometimes it will get to like 4 when we are accelerating or taking a sharp turn)
        if self.stuckTracker >= 15:
            retCmd = cmdIn #set the returned cmd to the cmd that we were originally given.
            self._prevCmd = None

        return retCmd