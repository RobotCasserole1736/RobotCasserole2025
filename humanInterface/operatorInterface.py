from utils.faults import Fault
from utils.signalLogging import addLog
from wpilib import XboxController
from Elevatorandmech.ElevatorandMechConstants import AlgaeWristState, ElevatorLevelCmd

class OperatorInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # controller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")
        self.ejectCoral = False
        self.autoIntakeCoral = False
        self.intakeAlgae = False
        self.ejectAlgae = False

        #elevator commands
        self.elevatorLevelCmd = ElevatorLevelCmd.NO_CMD
        self.elevManAdjCmd = 0.0

        self.algaeManipGround = False
        self.algaeManipStow = False
        self.algaeManipReef = False

        #addLog("scoreL1",lambda: self.L1,"Bool")
        #addLog("scoreL2",lambda: self.L2,"Bool")
        #addLog("scoreL3",lambda: self.L3,"Bool")
        #addLog("scoreL4",lambda: self.L4,"Bool")
        #addLog("elevManUp", lambda: self.elevManualUp, "Bool")
        #addLog("elevManDown", lambda: self.elevManualDown, "Bool")
        addLog("intakeAlgaeOpCmd", lambda: self.intakeAlgae, "Bool")
        #addLog("ejectAlgaeOpCmd", lambda: self.ejectAlgae, "Bool")
        #addLog("ejectCoral", lambda: self.ejectCoral, "Bool")
        addLog("autoIntakeCoral", lambda: self.autoIntakeCoral, "Bool")
    
    def update(self):
        # value of controller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions

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
            self.elevManAdjCmd = self.ctrl.getRightTriggerAxis() - self.ctrl.getLeftTriggerAxis() 


            self.intakeAlgae = self.ctrl.getLeftY() > 0.3
            self.ejectAlgae = self.ctrl.getLeftY() < -0.3
            self.ejectCoral = self.ctrl.getLeftBumper()

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
            self.elevatorLevelCmd = ElevatorLevelCmd.NO_CMD
            self.elevManAdjCmd = 0.0
            self.connectedFault.setFaulted()

    def getEjectCoral(self):
        return self.ejectCoral
    
    def getIntakeAlgae(self):
        return self.intakeAlgae
    
    def getEjectAlgae(self):
        return self.ejectAlgae

    def getAlgaeManipCmd(self):
        if self.algaeManipReef:
            return AlgaeWristState.REEF
        elif self.algaeManipGround:
            return AlgaeWristState.INTAKEOFFGROUND
        else:
            return AlgaeWristState.STOW

    def getElevCmd(self) -> ElevatorLevelCmd:
        return self.elevatorLevelCmd
    
    # Returns a manual offset to the elevator height
    # -1.0 is full down motion, 1.0 is full up motion
    def getElevManAdjCmd(self) -> float:
        return self.elevManAdjCmd

    def getAutoIntake(self):
        return self.autoIntakeCoral