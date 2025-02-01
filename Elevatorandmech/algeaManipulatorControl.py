from math import cos
from Elevatorandmech.ElevatorandMechConstants import ALGAE_ANGLE_ABS_POS_ENC_OFFSET
from utils.signalLogging import log
import wpilib
from wpimath.trajectory import TrapezoidProfile
from utils import faults
from utils.calibration import Calibration
from utils.constants import ALGAE_INT_CANID, ALGAE_WRIST_CANID, ALGAE_GAMEPIECE_CANID, ALGAE_ENC_PORT
from utils.singleton import Singleton
from enum import Enum
from utils.units import m2in, rad2Deg, deg2Rad
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from playingwithfusion import TimeOfFlight
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder
import math

class AlgaeWristState(Enum):
    DISABLED = 0
    INTAKEOFFGROUND = 1
    STOW = 2
    REEF = 3

#The most important question - why is this two separate things?? I think they should be combined
#the same button-type thing will be pressed to bring the algae manipulator up to angle, then start spinning it
class AlgaeWristControl(metaclass=Singleton):
    def __init__(self):
        #one important assumption we're making right now is that we don't need limits on the algae manipulator based on elevator height

        self.kG = Calibration(name="Algae kG", default=0.1, units="V/degErr")
        self.kS = Calibration(name="Algae kS", default=0.1, units="V/degErr")
        self.kV = Calibration(name="Algae kV", default=0.1, units="V/degErr")
        self.kP = Calibration(name="Algae kP", default=0.1, units="V/degErr")
        self.maxOutputV = Calibration(name="Algae Max Voltage", default=12.0, units="V")


        self.WristCurState = AlgaeWristState.DISABLED
        self.wristMotor = WrapperedSparkMax(ALGAE_WRIST_CANID, "AlgaeWristMotor", True, 10)
        self.curMotorVoltage = 0.0
        self.STOW_POS_DEG = Calibration(name="Stow Position", default = 0, units="deg")
        self.INTAKE_POS_DEG = Calibration(name="Intake Position", default = 0, units="deg")
        self.REEF_POS_DEG = Calibration(name="Reef Position", default = 0, units="deg")
        self.ABSOLUTE_OFFSET = ALGAE_ANGLE_ABS_POS_ENC_OFFSET
        self.curPosCmdDeg = self.STOW_POS_DEG.get()
        self.algaeAbsEnc = WrapperedThroughBoreHexEncoder(port=ALGAE_ENC_PORT, name="AlgaeEncOffset", mountOffsetRad=deg2Rad(self.ABSOLUTE_OFFSET))
        self.pos = AlgaeWristState
        self.disPos = 0
        self.inPos = 0
        self.stowPos = 0
        self.reefPos = 0
        self.maxAcc = 0
        self.maxVel = 0
        self.vel = 0
        #set the degree details for your goals. For stow, intake off gorund, etc. 
        #Also, the absolute offset will be a constant that you can set here or just have in constants 
    
    def setDesPos(self,desPosin):
        self.curPosCmdDeg = desPosin

    def getAngleDeg(self):
        return rad2Deg(self.algaeAbsEnc.getAngleRad())
    
    def update(self):
        
        #this is where you will figure out where you're trying to go. See tunerAngleControl.py for a simple answer
        #this could work if it's light enough. But you might have to go to something more like singerA
        if(self.pos == 0):
            self.desPos = self.disPos
        elif(self.pos == 1):
            self.desPos = self.inPos
        elif(self.pos == 2):
            self.desPos = self.stowPos
        elif(self.pos == 3):
            self.desPos = self.reefPos
        else:
            pass

        TrapezoidProfile.Constraints(self.maxVel, self.maxAcc)#check this alex
        TrapezoidProfile.State(self.desPos, self.vel)#i did this but i dont know if i need all of these 
        self.profile = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxVel, self.maxAcc))#check this alex



        self.algaeAbsEnc.update()
        
        actualPos = self.getAngleDeg()

        self.kG = math.cos(actualPos)

        err =  self.curPosCmdDeg - actualPos
        
        motorCmdV = err * self.kP.get()

        maxMag = self.maxOutputV.get()

        if(motorCmdV > maxMag):
            motorCmdV = maxMag
        elif(motorCmdV < -maxMag):
            motorCmdV = -maxMag
        
        self.wristMotor.setVoltage(motorCmdV)

        log("Algae Pos Des", self.curPosCmdDeg,"deg")
        log("Algae Pos Act", actualPos ,"deg")
        log("Algae Motor Cmd", motorCmdV, "V")
        
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