from enum import Enum
from math import cos
from Elevatorandmech.ElevatorandMechConstants import ALGAE_ANGLE_ABS_POS_ENC_OFFSET, AlgaeWristState
from utils.signalLogging import addLog
from utils import faults
from utils.calibration import Calibration
from utils.constants import ALGAE_INT_CANID, ALGAE_WRIST_CANID, ALGAE_GAMEPIECE_CANID, ALGAE_ENC_PORT
from utils.singleton import Singleton
from utils.units import m2in, rad2Deg, deg2Rad
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder


#The most important question - why is this two separate things?? I think they should be combined
#the same button-type thing will be pressed to bring the algae manipulator up to angle, then start spinning it
class AlgaeWristControl(metaclass=Singleton):
    def __init__(self):
        #one important assumption we're making right now is that we don't need limits on the algae manipulator based on elevator height

        #motor and encoder
        self.wristMotor = WrapperedSparkMax(ALGAE_WRIST_CANID, "AlgaeWristMotor", brakeMode=True, currentLimitA=20.0)
        self.algaeAbsEnc = WrapperedThroughBoreHexEncoder(port=ALGAE_ENC_PORT, name="Algae Wrist Enc", mountOffsetRad=deg2Rad(ALGAE_ANGLE_ABS_POS_ENC_OFFSET), dirInverted=True)

        #PID stuff calibrations
        self.kP = Calibration(name="Algae Wrist kP", default=.6, units="V/degErr")
        self.maxV = Calibration(name="Algae Wrist maxV", default=6.0, units="V")
        self.deadzone = Calibration(name="Algae Wrist deadzone", default=4.0, units="deg")

        #position calibrations... an angle in degrees. Assumingt 0 is horizontal, - is down, etc.  
        self.inPos = Calibration(name="Algae Wrist Barge Position", default = 80, units="deg")
        self.stowPos = Calibration(name="Algae Wrist Stow Position", default = 95, units="deg")
        self.reefPos = Calibration(name="Algae Wrist Reef Position", default = -20, units="deg")
        self.procPos = Calibration(name="Algae Wrist Processor Position", default = 10, units="deg")
        self.intakeOffGroundPos = Calibration(name="Algae Wrist Intake Off Ground Position", default = -20, units="deg")
        
        #positions
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()
        self.pos = AlgaeWristState.NOTHING

        addLog("Algae Wrist Desired Angle",lambda: self.curPosCmdDeg, "deg")
        addLog("Algae Wrist Actual Angle", lambda: rad2Deg(self.getAngleRad()), "deg")

    def setDesPos(self, desState : AlgaeWristState):
        #this is called in teleop periodic or autonomous to set the desired pos of algae manipulator
        self.curPosCmdDeg = self._posToDegrees(desState)

    def getAngleRad(self):
        return (self.algaeAbsEnc.getAngleRad())

    # Might optimize to accept 1 enum parameter for new position
    def _posToDegrees(self,pos:AlgaeWristState) -> float:
        self.pos = pos
        if(pos == AlgaeWristState.BARGE):
            return self.inPos.get()
        elif(pos == AlgaeWristState.REEF):
            return self.reefPos.get()
        elif(pos == AlgaeWristState.PROCESSOR):
            return self.procPos.get()
        elif(pos == AlgaeWristState.INTAKEOFFGROUND):
            return self.intakeOffGroundPos.get()
        else:
            return self.stowPos.get()

    def update(self):

        self.algaeAbsEnc.update()
        self.actualPos = rad2Deg(self.getAngleRad())

        if(self.algaeAbsEnc.isFaulted()):
            vCmd = 0.0 # faulted, so stop
        else:
            # Limited-output P control with deadzone
            err = self.curPosCmdDeg - self.actualPos
            if(abs(err) <= self.deadzone.get()):
                # in deadzone, no command
                vCmd = 0
            elif self.pos == AlgaeWristState.NOTHING:
                # No command, so keep voltage at zero
                vCmd = 0
            else:
                # Command and outside deadzone
                # P control with limit
                
                # Adjust error so that it's offset by the deadzone
                if(err>0):
                    err = err - self.deadzone.get()
                else:
                    err = err + self.deadzone.get()

                vCmd = self.kP.get() * err
                vCmd = min(self.maxV.get(), max(-self.maxV.get(), vCmd))

        self.wristMotor.setVoltage(vCmd)



class AlgeaIntakeControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommandState = False
        self.ejectCommandState = False
       
        self.algaeMotor = WrapperedSparkMax(ALGAE_INT_CANID, "AlgaeIntakeMotor", brakeMode=True, currentLimitA=10)

        self.intakeVoltageCal = Calibration("Algae Manipulator IntakeVoltage", 12, "V")
        self.ejectVoltageCal = Calibration("Algae Manipulator EjectVoltage", -12, "V")

        #addLog("Algae Manipulator intake cmd",lambda:self.intakeCommandState,"Bool")
        #addLog("Algae Manipulator  cmd",lambda:self.ejectCommandState,"Bool")
        #addLog("Has Game Piece", self.getHasGamePiece, "Bool")

    def update(self):

        if self.intakeCommandState:
            self.updateIntake(True)
        elif self.ejectCommandState:
            self.updateEject(True)
        else:
            self.algaeMotor.setVoltage(0)

    def setInput(self, intakeBool, ejectBool, algaeAngleEnum):
        self.intakeCommandState = intakeBool
        self.ejectCommandState = ejectBool

    def updateIntake(self, run):
        voltage = self.intakeVoltageCal.get()

        if run:
            self.algaeMotor.setVoltage(voltage)
        else: 
            self.algaeMotor.setVoltage(0)
    
    def updateEject(self, run):
        voltage = self.ejectVoltageCal.get()

        if run:
            self.algaeMotor.setVoltage(voltage)
        else:
            self.algaeMotor.setVoltage(0)