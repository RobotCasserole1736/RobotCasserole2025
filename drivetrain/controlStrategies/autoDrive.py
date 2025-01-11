from wpilib import Timer
from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import Trajectory
from drivetrain.controlStrategies.holonomicDriveController import HolonomicDriveController
from drivetrain.drivetrainCommand import DrivetrainCommand
from navigation.obstacleDetector import ObstacleDetector
from utils.signalLogging import addLog
from utils.singleton import Singleton
from navigation.repulsorFieldPlanner import RepulsorFieldPlanner
from navigation.navConstants import GOAL_1A, GOAL_1B, GOAL_2A, GOAL_2B, GOAL_3A, GOAL_3B, GOAL_4A, GOAL_4B, GOAL_5A, GOAL_5B, GOAL_6A, GOAL_6B
from drivetrain.drivetrainPhysical import MAX_DT_LINEAR_SPEED_MPS
from utils.allianceTransformUtils import transform
import math

# Maximum speed that we'll attempt to path plan at. Needs to be at least 
# slightly less than the maximum physical speed, so the robot can "catch up" 
# if it gets off the planned path
MAX_PATHPLAN_SPEED_MPS = 0.75 * MAX_DT_LINEAR_SPEED_MPS

class AutoDrive(metaclass=Singleton):
    def __init__(self):
        self._toSpeaker = False
        self._toPickup = False
        self.rfp = RepulsorFieldPlanner()
        self._trajCtrl = HolonomicDriveController("AutoDrive")
        self._telemTraj = []
        self._obsDet = ObstacleDetector()
        self._olCmd = DrivetrainCommand()
        self._prevCmd:DrivetrainCommand|None = None
        self._plannerDur:float = 0.0
        self.autoSpeakerPrevEnabled = False #This name might be a wee bit confusing. It just keeps track if we were in auto targeting the speaker last refresh.
        self.autoPickupPrevEnabled = False #This name might be a wee bit confusing. It just keeps track if we were in auto targeting the speaker last refresh.
        self.stuckTracker = 0 
        self._aOrB = "C"
        self._reefSide = -1
        self.prevPose = Pose2d()

        addLog("AutoDrive Proc Time", lambda:(self._plannerDur * 1000.0), "ms")


    def setRequest(self, aOrB, reefSide) -> None:
        self._aOrB = aOrB
        self._reefSide = reefSide
        #The following if statement is just logic to enable self.autoPrevEnabled when the driver enables an auto.
        #if self.autoSpeakerPrevEnabled == self._toSpeaker or self.autoPickupPrevEnabled != self._toPickup:
        #    self.stuckTracker = 0
        #We didn't want to deal with stuck code right now
        self.stuckTracker = 0
        self.autoPickupPrevEnabled = self._toPickup
        self.autoSpeakerPrevEnabled = self._toSpeaker
        

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

        retCmd = cmdIn # default - no auto driving

        for obs in self._obsDet.getObstacles(curPose):
            self.rfp.addObstacleObservation(obs)

        self.rfp._decayObservations()

        # Handle command changes
        if(self._aOrB == "A"):
            if self._reefSide == 1:
                self.rfp.setGoal(transform(GOAL_1A))
            elif self._reefSide == 2: 
                self.rfp.setGoal(transform(GOAL_2A))
            elif self._reefSide == 3: 
                self.rfp.setGoal(transform(GOAL_3A))
            elif self._reefSide == 4: 
                self.rfp.setGoal(transform(GOAL_4A))
            elif self._reefSide == 5: 
                self.rfp.setGoal(transform(GOAL_5A))
            elif self._reefSide == 6: 
                self.rfp.setGoal(transform(GOAL_6A))
            else:
                self.rfp.setGoal(None)
        elif (self._aOrB == "B"):
            if self._reefSide == 1:
                self.rfp.setGoal(transform(GOAL_1B))
            elif self._reefSide == 2: 
                self.rfp.setGoal(transform(GOAL_2B))
            elif self._reefSide == 3: 
                self.rfp.setGoal(transform(GOAL_3B))
            elif self._reefSide == 4: 
                self.rfp.setGoal(transform(GOAL_4B))
            elif self._reefSide == 5: 
                self.rfp.setGoal(transform(GOAL_5B))
            elif self._reefSide == 6: 
                self.rfp.setGoal(transform(GOAL_6B))
            else:
                self.rfp.setGoal(None)
        else:
            self.rfp.setGoal(None)

        # If being asked to auto-align somewhere, use the command from the dynamic path planner
        if(self._aOrB != "C" and self._reefSide != -1):
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
        #assume that we are either stuck or done if the counter reaches above 10. (sometimes it will get to like 4 when we are accelerating or taking a sharp turn)
        if self.stuckTracker >= 10:
            retCmd = cmdIn #set the returned cmd to the cmd that we were originally given.
            self._prevCmd = None

        return retCmd