from dataclasses import dataclass
import wpilib
from wpimath.units import feetToMeters, degreesToRadians
from wpimath.geometry import Pose2d
from wrappers.casserolePhotonCamera import PhotonCamera
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.photonCamera import setVersionCheckEnabled
from utils.fieldTagLayout import FieldTagLayout
from utils.faults import Fault
from utils.signalLogging import addLog

# Describes one on-field pose estimate from the a camera at a specific time.
@dataclass
class CameraPoseObservation:
    time:float
    estFieldPose:Pose2d
    xyStdDev:float=1.0 # std dev of error in measurment, units of meters.
    rotStdDev:float=degreesToRadians(99999.0) # std dev of measurement, in units of radians

# Sort Tags by location
REEF_TAG_IDS = [6,7,8,9,10,11,17,18,19,20,21,22]
BARGE_TAG_IDS = [4,5,14,15]
PROCESSOR_TAG_IDS = [3,16]
HUMAN_STATION_TAG_IDS = [1,2,12,13]

# Wrappers photonvision to:
# 1 - resolve issues with target ambiguity (two possible poses for each observation)
# 2 - Convert pose estimates to the field
# 3 - Handle recording latency of when the image was actually seen
class WrapperedPoseEstPhotonCamera:
    def __init__(self, camName, robotToCam):
        setVersionCheckEnabled(False)

        self.name = camName
        self.cam = PhotonCamera(camName)

        self.disconFault = Fault(f"Camera {camName} not sending data")
        self.timeoutSec = 1.0
        self.poseEstimates:list[CameraPoseObservation] = []
        self.robotToCam = robotToCam
        self.lastLatency = 0.0
        self.updateDuration = 0.0
        addLog(f"PoseCam {self.name} latency", lambda:self.lastLatency, "sec")
        addLog(f"PoseCam {self.name} Duration", lambda:self.updateDuration, "ms")
        self.prevTimestampSec = 0.0
        self.singleTagModeTagList = None #not currently used


        self.lastCaptureTime = wpilib.Timer.getFPGATimestamp()
        self.CAP_PERIOD_SEC = 0.025
        #addLog("Camera Only Position", self.poseEstimates)
        #addLog("FPGA Timestamp", wpilib.Timer.getFPGATimestamp(), "sec")

    def setSingleTagMode(self, tag:list[int]|None):
        self.singleTagModeTagList = tag

    def update(self, prevEstPose:Pose2d):

        startTime = wpilib.Timer.getFPGATimestamp()

        self.poseEstimates = []
        self.lastLatency = 0.0

        if(not self.cam.isConnected()):
            # Faulted - no estimates, just return.
            self.disconFault.setFaulted()
            return
        
        # Periodically trigger a photo capture
        if(wpilib.DriverStation.isEnabled()):
            if( (startTime - self.lastCaptureTime) > self.CAP_PERIOD_SEC):
                self.lastCaptureTime = startTime
                self.cam.takeOutputSnapshot()
        

        # Grab whatever the camera last reported for observations in all camera frames
        res = self.cam.getLatestResult()

        if (res.getTimestampSeconds() != self.prevTimestampSec): # Only process if new timestamp
            resIdx = 0
            self.prevTimestampSec = res.getTimestampSeconds()
            obsTime = res.getTimestampSeconds()
            self.lastLatency = wpilib.Timer.getFPGATimestamp() - obsTime
            self.disconFault.setNoFault()


            # Process each target.
            # Each target has multiple solutions for where you could have been at on the field
            # when you observed it
            # (https://docs.wpilib.org/en/stable/docs/software/vision-processing/
            # apriltag/apriltag-intro.html#d-to-3d-ambiguity)
            # We want to select the best possible pose per target
            # We should also filter out targets that are too far away, and poses which
            # don't make sense.
            for tgtIdx, target in enumerate(res.getTargets()):
                # Transform both poses to on-field poses
                tgtID = target.getFiducialId()

                if tgtID >= 0:
                    # Only handle valid ID's
                    tagFieldPose = FieldTagLayout().lookup(tgtID)

                    if tagFieldPose is not None:
                        # Only handle known tags
                        poseCandidates:list[Pose2d] = []
                        if target.getPoseAmbiguity() <= 0.05:
                            # Only use very non-ambiguous estimates, throw the ambiguous ones out
                            poseCandidates.append(
                                self._toFieldPose(tagFieldPose, target.getBestCameraToTarget())
                            )
                            poseCandidates.append(
                                self._toFieldPose(tagFieldPose, target.getAlternateCameraToTarget())
                            )


                        # Filter candidates in this frame to only the valid ones
                        filteredCandidates:list[Pose2d] = []
                        for poseCandidate in poseCandidates:

                            # True if we're either not in single tag mode, 
                            # or (if we are) the tag matches
                            matchesSingleTag = (self.singleTagModeTagList is None or tgtID in self.singleTagModeTagList)

                            # Discard the bad tags on our practice field at least
                            isHorrible = tgtID in HUMAN_STATION_TAG_IDS or tgtID in BARGE_TAG_IDS

                            # is on field
                            onField = self._poseIsOnField(poseCandidate)
                            closeEnough = self._closeEnoughToCamera(target)
                            # Add other filter conditions here
                            if onField and closeEnough and not isHorrible and matchesSingleTag:
                                filteredCandidates.append(poseCandidate)


                        # Pick the candidate closest to the last estimate
                        bestCandidate:(Pose2d|None) = None
                        bestCandidateDist = 99999999.0
                        for poseCandidate in filteredCandidates:
                            delta = (poseCandidate - prevEstPose).translation().norm()
                            if delta < bestCandidateDist:
                                # This candidate is better, use it
                                bestCandidate = poseCandidate
                                bestCandidateDist = delta


                        # Finally, add our best candidate the list of pose observations
                        if bestCandidate is not None:
                            # TODO: we can probably get better standard deviations than just
                            # assuming the default. Check out what 254 did in 2024:
                            # https://github.com/Team254/FRC-2024-Public/blob/040f653744c9b18182be5f6bc51a7e505e346e59/src/main/java/com/team254/frc2024/subsystems/vision/VisionSubsystem.java#L381
                            
                            # First pass - we trust reef tags more
                            stdDev = 1.0 if tgtID in REEF_TAG_IDS else 3.0
                            
                            self.poseEstimates.append(
                                CameraPoseObservation(obsTime, 
                                                    bestCandidate, 
                                                    xyStdDev=stdDev)
                            )


        endTime = wpilib.Timer.getFPGATimestamp()
        self.updateDuration = (endTime - startTime)*1000.0

    def getPoseEstimates(self):
        return self.poseEstimates

    def _toFieldPose(self, tgtPose, camToTarget):
        camPose = tgtPose.transformBy(camToTarget.inverse())
        return camPose.transformBy(self.robotToCam.inverse()).toPose2d()

    # Returns true of a pose is on the field, false if it's outside of the field perimieter
    def _poseIsOnField(self, pose: Pose2d):
        trans = pose.translation()
        x = trans.X()
        y = trans.Y()
        inY = 0.0 <= y <= feetToMeters(27.0)
        inX = 0.0 <= x <= feetToMeters(54.0)
        return inX and inY
    
    def _closeEnoughToCamera(self, target: PhotonTrackedTarget):
        return target.getBestCameraToTarget().translation().norm() <= 2.0
    
