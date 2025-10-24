from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController
from wpilib import DriverStation
from utils.calibration import Calibration

class DriverInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # contoller
        ctrlIdx = 0
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Driver XBox controller ({ctrlIdx}) unplugged")

        # Drivetrain motion commands
        self.velXCmd = 0
        self.velYCmd = 0
        self.velTCmd = 0

        self.robotRelativeSlowdown = Calibration(name="Robot Relative Slowdown", default=.5, units="%")

        # Driver motion rate limiters - enforce smoother driving
        self.velXSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)
        self.velYSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)
        self.velTSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_ROTATE_ACCEL_RAD_PER_SEC_2)

        # Navigation commands
        self.autoDriveCmd = False
        self.autoSteerToAlgaeProcessor = False
        self.autoSteerDownfield = False

        # Utility - reset to zero-angle at the current pose
        self.gyroResetCmd = False
        #utility - use robot-relative commands
        self.robotRelative = False

        self.autoSteerEnable = True

        self.ejectCoral = False

        # Logging
        addLog("DI FwdRev Cmd", lambda: self.velXCmd, "mps")
        addLog("DI Strafe Cmd", lambda: self.velYCmd, "mps")
        addLog("DI Rot Cmd", lambda: self.velTCmd, "radps")
        addLog("DI AutoSteer Enable", lambda: self.autoSteerEnable, "radps")
        #addLog("DI gyroResetCmd", lambda: self.gyroResetCmd, "bool")
        #addLog("DI autoDriveToSpeaker", lambda: self.autoDriveToSpeaker, "bool")
        #addLog("DI autoDriveToPickup", lambda: self.autoDriveToPickup, "bool")

    def update(self):
        # value of contoller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            vXJoyRaw = self.ctrl.getLeftY() * -1
            vYJoyRaw = self.ctrl.getLeftX() * -1
            vRotJoyRaw = self.ctrl.getRightX() * -1

            self.robotRelative = self.ctrl.getLeftBumper()

            if not self.robotRelative:
                # Correct for alliance
                if onRed():
                    vXJoyRaw *= -1.0
                    vYJoyRaw *= -1.0

            # deadband
            vXJoyWithDeadband = applyDeadband(vXJoyRaw, 0.05)
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.05)
            vRotJoyWithDeadband = applyDeadband(vRotJoyRaw, 0.05)

            # TODO - if the driver wants a slow or sprint button, add it here.
            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.47

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * MAX_ROTATE_SPEED_RAD_PER_SEC * 0.8

            if self.robotRelative:
                velCmdXRaw *= self.robotRelativeSlowdown.get()
                velCmdYRaw *= self.robotRelativeSlowdown.get()
                velCmdRotRaw *= self.robotRelativeSlowdown.get()

            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)

            self.gyroResetCmd = self.ctrl.getAButton()

            self.autoDriveCmd = self.ctrl.getBButton()
            self.autoSteerToAlgaeProcessor = self.ctrl.getXButton()
            self.autoSteerDownfield = self.ctrl.getYButton()

            self.ejectCoral = self.ctrl.getRightTriggerAxis() > .5

            if(self.ctrl.getBackButton()):
                self.autoSteerEnable = False
            elif(self.ctrl.getStartButton()):
                self.autoSteerEnable = True
            else:
                pass

            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.velXCmd = 0.0
            self.velYCmd = 0.0
            self.velTCmd = 0.0
            self.gyroResetCmd = False
            self.autoDriveCmd = False
            self.robotRelative = False
            self.createDebugObstacle = False
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()

    def getCmd(self) -> DrivetrainCommand:
        retval = DrivetrainCommand()
        retval.velX = self.velXCmd
        retval.velY = self.velYCmd
        retval.velT = self.velTCmd
        return retval

    def getAutoDrive(self) -> bool:
        return self.autoDriveCmd
    
    def getAutoSteerEnable(self) -> bool:
        return self.autoSteerEnable

    def getAutoSteerToAlgaeProcessor(self) -> bool:
        return self.autoSteerToAlgaeProcessor
    
    def getAutoSteerDownfield(self) -> bool:
        return self.autoSteerDownfield

    def getGyroResetCmd(self) -> bool:
        return self.gyroResetCmd

    def getCreateObstacle(self) -> bool:
        return self.createDebugObstacle

    def getEjectCoral(self) -> bool:
        return self.ejectCoral

    def getRobotRelative(self):
        return self.robotRelative