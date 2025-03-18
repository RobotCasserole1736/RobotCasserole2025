from Elevatorandmech.ElevatorandMechConstants import AlgaeWristState, ElevatorLevelCmd, CoralManState
from utils.faults import Fault
from utils.signalLogging import addLog
from wpilib import DriverStation, XboxController
from utils.mapLookup2d import MapLookup2D



class OperatorInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self) -> None:
        # controller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")
        self.coralCmd = CoralManState.DISABLED
        self.intakeAlgae = False
        self.ejectAlgae = False



        #elevator commands
        self.elevatorLevelCmd = ElevatorLevelCmd.NO_CMD
        self.elevManAdjCmd = 0.0
        self.elevReset = False

        # Mostly on-off, but with a bit of controllability near the center
        # Input: joystick value (-1.0 to 1.0)
        # Output: Frac of max elevator manual adjust voltage
        self.elevManCmdLookup = MapLookup2D([
            (-1.0,  -1.0),
            (-0.4,  -1.0),
            (-0.1,   0.0), #deadzone
            (0.0,    0.0),
            (0.1,    0.0), #deadzone
            (0.4,    1.0),
            (1.0,    1.0)
        ])
        
        #Feedback for elevator being blocked while motion commanded
        self.elevBlocked = False

        self.algaeManipCmd = AlgaeWristState.STOW

        #addLog("elevManUp", lambda: self.elevManualUp, "Bool")
        #addLog("elevManDown", lambda: self.elevManualDown, "Bool")
        #addLog("intakeAlgaeOpCmd", lambda: self.intakeAlgae, "Bool")
        #addLog("ejectAlgaeOpCmd", lambda: self.ejectAlgae, "Bool")
        #addLog("ejectCoral", lambda: self.ejectCoral, "Bool")
        #addLog("autoIntakeCoral", lambda: self.autoIntakeCoral, "Bool")

    def update(self) -> None:
        # value of controller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions

            elevMotionCommanded = False

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
            self.elevManAdjCmd = self.elevManCmdLookup.lookup(self.ctrl.getLeftY() * -1.0) 

            # Give haptic feedback if the elevator is blocked but some level is commanded.
            elevMotionCommanded = self.elevatorLevelCmd != ElevatorLevelCmd.NO_CMD and self.elevManAdjCmd != 0.0
            if(elevMotionCommanded and self.elevBlocked):
                self.ctrl.setRumble(XboxController.RumbleType.kBothRumble, 1.0)
            else:
                self.ctrl.setRumble(XboxController.RumbleType.kBothRumble, 0.0)

            if self.ctrl.getRightTriggerAxis() > .2: #Currently Prioritizes right trigger 
                self.coralCmd = CoralManState.EJECTING
            elif self.ctrl.getLeftTriggerAxis() > .2:
                self.coralCmd = CoralManState.INTAKING
            else:
                self.coralCmd = CoralManState.DISABLED

            if self.ctrl.getRightBumper():
                self.ejectAlgae = True
                self.intakeAlgae = False
            elif self.ctrl.getLeftBumper():
                self.intakeAlgae = True 
                self.ejectAlgae = False
            else:
                self.ejectAlgae = False
                # But leave intake the same

            # Set Algae Manipulator command
            # Dpad down = Processor Score Position
            if 135 < self.ctrl.getPOV() < 225:
                self.algaeManipCmd = AlgaeWristState.PROCESSOR
                self.elevatorLevelCmd = ElevatorLevelCmd.L1
            #right joystick up = barge shot
            elif self.ctrl.getRightY() < -.5:
                self.algaeManipCmd = AlgaeWristState.BARGE
                self.elevatorLevelCmd = ElevatorLevelCmd.BARGE
            #right joystick down = intake off ground
            elif self.ctrl.getRightY() > .5:
                self.algaeManipCmd = AlgaeWristState.INTAKEOFFGROUND
                self.elevatorLevelCmd = ElevatorLevelCmd.L1
            # Dpad right = Stow Position
            elif 45 < self.ctrl.getPOV() < 135:
                self.algaeManipCmd = AlgaeWristState.STOW
            # Dpad left = Reef Position, L2
            elif 225 < self.ctrl.getPOV() < 315:
                self.algaeManipCmd = AlgaeWristState.REEF
                self.elevatorLevelCmd = ElevatorLevelCmd.AL2
            #Dpad up = Reef position, L3
            elif 315 < self.ctrl.getPOV() < 360 or 0 <= self.ctrl.getPOV() < 45:
                self.algaeManipCmd = AlgaeWristState.REEF
                self.elevatorLevelCmd = ElevatorLevelCmd.AL3
            else:
                self.algaeManipCmd = AlgaeWristState.NOTHING

            self.elevReset = self.ctrl.getBackButton()

            self.connectedFault.setNoFault()

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.autoIntakeCoral = False
            self.ejectCoral = False
            self.intakeAlgae = False
            self.algaeManipCmd = AlgaeWristState.STOW
            self.elevatorLevelCmd = ElevatorLevelCmd.NO_CMD
            self.elevManAdjCmd = 0.0
            self.elevReset = False
            if(DriverStation.isFMSAttached()):
                self.connectedFault.setFaulted()
                
    def getCoralCmd(self) -> CoralManState:
        return self.coralCmd

    def getIntakeAlgae(self) -> bool:
        return self.intakeAlgae

    def getEjectAlgae(self) -> bool:
        return self.ejectAlgae

    def getAlgaeManipCmd(self) -> AlgaeWristState:
        return self.algaeManipCmd

    def getElevCmd(self) -> ElevatorLevelCmd:
        return self.elevatorLevelCmd

    # Returns a manual offset to the elevator height
    # -1.0 is full down motion, 1.0 is full up motion
    def getElevManAdjCmd(self) -> float:
        return self.elevManAdjCmd

    def getElevReset(self):
        return self.elevReset
    
    def setElevatorBlocked(self, isBlocked):
        self.elevBlocked = isBlocked