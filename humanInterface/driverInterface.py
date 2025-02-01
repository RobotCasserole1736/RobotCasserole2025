from enum import Enum
from Elevatorandmech.ElevatorandMechConstants import ElevatorLevelCmd
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController

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

        # Driver motion rate limiters - enforce smoother driving
        self.velXSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)
        self.velYSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)
        self.velTSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_ROTATE_ACCEL_RAD_PER_SEC_2)

        # Navigation commands
        self.autoDrive = False
        self.createDebugObstacle = False

        # Utility - reset to zero-angle at the current pose
        self.gyroResetCmd = False
        #elevator commands
        self.elevatorLevelCmd = ElevatorLevelCmd.NO_CMD
        self.elevManAdjCmd = 0.0

        # Logging
        addLog("DI FwdRev Cmd", lambda: self.velXCmd, "mps")
        addLog("DI Strafe Cmd", lambda: self.velYCmd, "mps")
        addLog("DI Rot Cmd", lambda: self.velTCmd, "radps")
        addLog("DI GyroResetCmd", lambda: self.gyroResetCmd, "bool")
        addLog("DI AutoDrive", lambda: self.autoDrive, "bool")

    def update(self):
        # value of contoller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            vXJoyRaw = self.ctrl.getLeftY() * -1
            vYJoyRaw = self.ctrl.getLeftX() * -1
            vRotJoyRaw = self.ctrl.getRightX() * -1

            # Correct for alliance
            if onRed():
                vXJoyRaw *= -1.0
                vYJoyRaw *= -1.0

            # deadband
            vXJoyWithDeadband = applyDeadband(vXJoyRaw, 0.05)
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.05)
            vRotJoyWithDeadband = applyDeadband(vRotJoyRaw, 0.05)

            # TODO - if the driver wants a slow or sprint button, add it here.
            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.7
            #slowMult = 1.0

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * MAX_ROTATE_SPEED_RAD_PER_SEC * 0.8

            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)

            # Elevator Commands
            self.elevatorLevelCmd = ElevatorLevelCmd.NO_CMD # default to no command
            if(self.ctrl.getXButton()):
                self.elevatorLevelCmd = ElevatorLevelCmd.L1 
            elif(self.ctrl.getAButton()):
                self.elevatorLevelCmd = ElevatorLevelCmd.L2
            elif(self.ctrl.getBButton()):
                self.elevatorLevelCmd = ElevatorLevelCmd.L3
            elif(self.ctrl.getYButton()):
                self.elevatorLevelCmd = ElevatorLevelCmd.L4

            self.L1 = self.ctrl.getXButton()
            self.L2 = self.ctrl.getAButton()
            self.L3 = self.ctrl.getBButton()
            self.L4 = self.ctrl.getYButton()

            self.elevManAdjCmd = self.ctrl.getRightTriggerAxis() - self.ctrl.getLeftTriggerAxis() 
            
            self.gyroResetCmd = self.ctrl.getAButton()

            self.autoDrive = self.ctrl.getBButton()
            self.createDebugObstacle = self.ctrl.getYButtonPressed()

            self.climberExtendV = applyDeadband(self.ctrl.getLeftTriggerAxis(),.1) * 12
            self.climberRetractV = applyDeadband(self.ctrl.getRightTriggerAxis(),.1) *-12

            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.velXCmd = 0.0
            self.velYCmd = 0.0
            self.velTCmd = 0.0
            self.gyroResetCmd = False
            self.autoDrive = False
            self.createDebugObstacle = False
            self.elevatorLevelCmd = ElevatorLevelCmd.NO_CMD
            self.elevManAdjCmd = 0.0
            self.connectedFault.setFaulted()





    def getCmd(self) -> DrivetrainCommand:
        retval = DrivetrainCommand()
        retval.velX = self.velXCmd
        retval.velY = self.velYCmd
        retval.velT = self.velTCmd
        return retval

    def getAutoDrive(self) -> bool:
        return self.autoDrive

    def getGyroResetCmd(self) -> bool:
        return self.gyroResetCmd

    def getCreateObstacle(self) -> bool:
        return self.createDebugObstacle
    
    def getElevCmd(self) -> ElevatorLevelCmd:
        return self.elevatorLevelCmd
    
    # Returns a manual offset to the elevator height
    # -1.0 is full down motion, 1.0 is full up motion
    def getElevManAdjCmd(self) -> float:
        return self.elevManAdjCmd
