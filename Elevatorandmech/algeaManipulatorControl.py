import wpilib
import wpimath 
from utils import faults
from utils.calibration import Calibration
from utils.constants import ALGAE_INT_CANID, ALGAE_WRIST_CANID, ALGAE_GAMEPIECE_CANID
from utils.singleton import Singleton
from enum import Enum
from utils.units import m2in
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from playingwithfusion import TimeOfFlight
class AlgaeWristState(Enum):
    DISABLED = 4
    INTAKEOFFGROUND = 5
    STOW = 6
    REEF = 7

#The most important question - why is this two separate things?? I think they should be combined
#the same button-type thing will be pressed to bring the algae manipulator up to angle, then start spinning it
class AlgaeWristControl(metaclass=Singleton):
    def __init__(self):
        #one important assumption we're making right now is that we don't need limits on the algae manipulator based on elevator height
        
        self.WristCurState = AlgaeWristState.DISABLED
        self.wristMotor = WrapperedSparkMax(ALGAE_WRIST_CANID, "AlgaeWristMotor", True, 10)
        self.curMotorVoltage = 0.0
        self.STOW_POS_DEG = -53.0 #CHANGE
        self.INTAKE_POS_POS = 112 #CHANGE
        self.REEF_POS_DEG = 0.0 #CHANGE
        self.ABSOLUTE_OFFSET = 0

        #set the degree details for your goals. For stow, intake off gorund, etc. 
        #Also, the absolute offset will be a constant that you can set here or just have in constants 

    def update(self):
        
        #this is where you will figure out where you're trying to go. See tunerAngleControl.py for a simple answer
        #this could work if it's light enough. But you might have to go to something more like singerAngleControl.py
        pass
        
        
     
    def setGoal(self):
        #here is where you will want to set which angle you're trying to go to. See 2024 tunerAngleControl.py
        pass
         
class AlgeaIntakeControl(metaclass=Singleton):

    def __init__(self):
        #initialize all of your variables, spark maxes, etc. 
        self.intakeCommandState = False
        self.ejectCommandState = False

        
        self.algaeMotor = WrapperedSparkMax(ALGAE_INT_CANID, "AlgaeIntakeMotor", True)

        self.intakeVoltageCal = Calibration("IntakeVoltage", 12, "V")
        self.ejectVoltageCal = Calibration("EjectVoltage", 12, "V")
        self.holdVoltageCal = Calibration("HoldVoltage", 12, "V")

        self.tofSensor = TimeOfFlight(ALGAE_GAMEPIECE_CANID)
        self.tofSensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 24)
        self.tofSensor.setRangeOfInterest(6, 6, 10, 10)  # fov for sensor # game piece sensor empty for now
        # LOOK AT THIS LATER YOU SOFTWARE PEOPLE YOU BETTER LOOK AT THIS

        self.gamePiecePresentCal = Calibration("AlgaePresentThresh", 24, "in")


        self.disconTOFFault = faults.Fault("Singer TOF Sensor is Disconnected")

    def update(self):
        
        gamepieceDistSensorMeas = m2in(self.tofSensor.getRange() / 1000.0)
        self.disconTOFFault.set(self.tofSensor.getFirmwareVersion() == 0)
        #update TOF (is it faulted? Is there a gamepiece being seen (see checkGamePiece function)?)
        if(self.disconTOFFault.isActive):
            # Gamepiece Sensor Faulted - assume we don't have a gamepiece
            self.hasGamePiece = False
        else:
            if gamepieceDistSensorMeas < self.gamePiecePresentCal.get():
                self.hasGamePiece = True
            else: 
                self.hasGamePiece = False

        if  self.intakeCommandState:
            self.updateIntake(True)
        elif self.ejectCommandState:
            self.updateEject(True)
        elif self.hasGamePiece:
            self.updateHold(True)

        else:
            self.algaeMotor.setVoltage(0)

    def setInput(self, intakeBool, ejectBool):
        self.intakeCommandState = intakeBool
        self.ejectCommandState = ejectBool

    def updateIntake(self, run):
        if run:
            self.algaeMotor.setVoltage(self.intakeVoltageCal.get())
        else: 
            self.algaeMotor.setVoltage(0)
    

    def updateEject(self, run):
        if run:
            self.algaeMotor.setVoltage(self.ejectVoltageCal.get())
        else:
            self.algaeMotor.setVoltage(0)

    def updateHold(self, run):
        if run:
            self.algaeMotor.setVoltage(self.holdVoltageCal.get())
        else:
            self.algaeMotor.setVoltage(0)