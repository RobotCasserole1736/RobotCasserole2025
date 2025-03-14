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
        self.wristMotor = WrapperedSparkMax(ALGAE_WRIST_CANID, "AlgaeWristMotor", True)
        self.algaeAbsEnc = WrapperedThroughBoreHexEncoder(port=ALGAE_ENC_PORT, name="Algae Wrist Enc", mountOffsetRad=deg2Rad(ALGAE_ANGLE_ABS_POS_ENC_OFFSET), dirInverted=True)

        #PID stuff calibrations
        self.kP = Calibration(name="Algae Wrist kP", default=.6, units="V/degErr")
        self.maxV = Calibration(name="Algae Wrist maxV", default=4.0, units="V")
        self.deadzone = Calibration(name="Algae Wrist deadzone", default=8.0, units="deg")

        #position calibrations... an angle in degrees. Assumingt 0 is horizontal, - is down, etc.  
        self.inPos = Calibration(name="Algae Wrist Intake Position", default = -15, units="deg")
        self.stowPos = Calibration(name="Algae Wrist Stow Position", default = -90, units="deg")
        self.reefPos = Calibration(name="Algae Wrist Reef Position", default = -70, units="deg")
        
        #positions
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()
        self.pos = AlgaeWristState.NOTHING

        #addLog("Algae Wrist Desired Angle",lambda: self.curPosCmdDeg, "deg")
        #addLog("Algae Wrist Actual Angle", lambda: rad2Deg(self.getAngleRad()), "deg")

    def setDesPos(self, desState : AlgaeWristState):
        #this is called in teleop periodic or autonomous to set the desired pos of algae manipulator
        self.curPosCmdDeg = self._posToDegrees(desState)

    def getAngleRad(self):
        return (self.algaeAbsEnc.getAngleRad())

    # Might optimize to accept 1 enum parameter for new position
    def _posToDegrees(self,pos:AlgaeWristState) -> float:
        self.pos = pos
        if(pos == AlgaeWristState.INTAKEOFFGROUND):
            return self.inPos.get()
        elif(pos == AlgaeWristState.REEF):
            return self.reefPos.get()
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
        self.flip = False
        #self.flip will flip the intake/eject voltages if the algae wrist is at a high angle
        
        self.algaeMotor = WrapperedSparkMax(ALGAE_INT_CANID, "AlgaeIntakeMotor", brakeMode=True, currentLimitA=10)

        self.intakeVoltageCal = Calibration("Algae Manipulator IntakeVoltage", -12, "V")
        self.ejectVoltageCal = Calibration("Algae Manipulator EjectVoltage", 12, "V")

        #addLog("Algae Manipulator intake cmd",lambda:self.intakeCommandState,"Bool")
        #addLog("Algae Manipulator  cmd",lambda:self.ejectCommandState,"Bool")
        #addLog("Has Game Piece", self.getHasGamePiece, "Bool")

    def update(self):

        if self.intakeCommandState:
            self.updateIntake(True, self.flip)
        elif self.ejectCommandState:
            self.updateEject(True, self.flip)
        else:
            self.algaeMotor.setVoltage(0)

    def setInput(self, intakeBool, ejectBool, algaeAngleEnum):
        self.intakeCommandState = intakeBool
        self.ejectCommandState = ejectBool
        if AlgaeWristState.INTAKEOFFGROUND == algaeAngleEnum:
            self.flip=True
        elif AlgaeWristState.NOTHING == algaeAngleEnum:
            pass
            #we want to pass here to preserve flip from last loop
        else:
            self.flip=False

    def updateIntake(self, run, flip):
        if flip:
            voltage = self.intakeVoltageCal.get() * -1
        else:
            voltage = self.intakeVoltageCal.get()

        if run:
            self.algaeMotor.setVoltage(voltage)
        else: 
            self.algaeMotor.setVoltage(0)
    
    def updateEject(self, run, flip):
        if flip:
            voltage = self.ejectVoltageCal.get() * -1
        else:
            voltage = self.ejectVoltageCal.get()

        if run:
            self.algaeMotor.setVoltage(voltage)
        else:
            self.algaeMotor.setVoltage(0)