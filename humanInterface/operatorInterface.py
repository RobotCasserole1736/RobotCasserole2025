from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController

class OperatorInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # controller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")
        self.ejectCoral = False
        self.intakeAlgae = False
        self.ejectAlgae = False
        #we are going to use elevManual commands as a if-pressed, make goal go up by __ m
        self.elevManualUp = False
        self.elevManualDown = False
        self.L1 = False
        self.L2 = False
        self.L3 = False
        self.L4 = False

        # Logging

    def update(self):
        # value of controller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions

            self.L1 = self.ctrl.getXButton()
            self.L2 = self.ctrl.getAButton()
            self.L3 = self.ctrl.getBButton()
            self.L4 = self.ctrl.getYButton()

            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.ejectCoral = False
            self.intakeAlgae = False
            self.ejectAlgae = False
            self.elevManualUp = False
            self.elevManualDown = False
            self.L1 = False
            self.L2 = False
            self.L3 = False
            self.L4 = False
            self.connectedFault.setFaulted()

    def getEjectCoral(self):
        return self.ejectCoral
    
    def getIntakeAlgae(self):
        return self.intakeAlgae
    
    def getEjectAlgae(self):
        return self.ejectAlgae
    
    def getL1(self):
        return self.L1
    
    def getL2(self):
        return self.L2

    def getL3(self):
        return self.L3
    
    def getL4(self):
        return self.L4
    
    def getElevManUp(self):
        return self.elevManualUp
    
    def getElevManDown(self):
        return self.elevManualDown

"""def getCmd(self):
    return self.Cmd
"""