from utils.faults import Fault
from utils.signalLogging import addLog
from wpilib import XboxController
from Elevatorandmech.ElevatorandMechConstants import AlgaeWristState

class OperatorInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # controller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")
        self.ejectCoral = False
        self.autoIntakeCoral = True
        self.intakeAlgae = False
        self.ejectAlgae = False
        #we are going to use elevManual commands as a if-pressed, make goal go up by __ m
        self.elevManualUp = False
        self.elevManualDown = False
        self.L1 = False
        self.L2 = False
        self.L3 = False
        self.L4 = False

        self.algaeManipDisabledPos = False
        self.algaeManipGround = False
        self.algaeManipStow = False
        self.algaeManipReef = False

        #addLog("scoreL1",lambda: self.L1,"Bool")
        #addLog("scoreL2",lambda: self.L2,"Bool")
        #addLog("scoreL3",lambda: self.L3,"Bool")
        #addLog("scoreL4",lambda: self.L4,"Bool")
        #addLog("elevManUp", lambda: self.elevManualUp, "Bool")
        #addLog("elevManDown", lambda: self.elevManualDown, "Bool")
        addLog("intakeAlgae", lambda: self.intakeAlgae, "Bool")
        addLog("ejectAlgae", lambda: self.ejectAlgae, "Bool")
        #addLog("ejectCoral", lambda: self.ejectCoral, "Bool")
        #addLog("autoIntakeCoral", lambda: self.autoIntakeCoral, "Bool")
    
    def update(self):
        # value of controller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions

            self.L1 = self.ctrl.getXButton()
            self.L2 = self.ctrl.getAButton()
            self.L3 = self.ctrl.getBButton()
            self.L4 = self.ctrl.getYButton()
            self.elevManualUp = self.ctrl.getRightTriggerAxis()
            self.elevManualDown = self.ctrl.getLeftTriggerAxis()

            self.intakeAlgae = self.ctrl.getLeftY() > 0.3
            self.ejectAlgae = self.ctrl.getLeftY() < -0.3
            # self.ejectCoral = True if self.ctrl.getPOV() != -1 else False
            self.ejectCoral = self.ctrl.getLeftBumper()

            # Dpad up
            self.algaeManipDisabledPos =  self.ctrl.getPOV() < 45 or self.ctrl.getPOV() > 315
            # Dpad right
            self.algaeManipGround = 45 < self.ctrl.getPOV() < 135
            # Dpad down
            self.algaeManipStow = 135 < self.ctrl.getPOV() < 225
            # Dpad left
            self.algaeManipReef = 225 < self.ctrl.getPOV() < 315

            if self.ctrl.getBackButtonPressed():
                if self.autoIntakeCoral:
                    self.autoIntakeCoral = False
                else:
                    self.autoIntakeCoral = True

            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.autoIntakeCoral = False
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

    def getAlgaeManipCmd(self):
        if self.algaeManipReef:
            return AlgaeWristState.REEF
        elif self.algaeManipStow:
            return AlgaeWristState.STOW
        elif self.algaeManipGround:
            return AlgaeWristState.INTAKEOFFGROUND
        else:
            return AlgaeWristState.DISABLED

    def getElevManUp(self):
        return self.elevManualUp
    
    def getElevManDown(self):
        return self.elevManualDown
    
    def getAutoIntake(self):
        return self.autoIntakeCoral