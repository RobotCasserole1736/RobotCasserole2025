from enum import Enum
from math import cos
from playingwithfusion import TimeOfFlight
from Elevatorandmech.ElevatorandMechConstants import ALGAE_ANGLE_ABS_POS_ENC_OFFSET, ALGAE_GEARBOX_GEAR_RATIO, AlgaeWristState
from utils.signalLogging import addLog, log
from wpimath.trajectory import TrapezoidProfile
from utils import faults
from utils.calibration import Calibration
from utils.constants import ALGAE_INT_CANID, ALGAE_WRIST_CANID, ALGAE_GAMEPIECE_CANID, ALGAE_ENC_PORT
from utils.singleton import Singleton
from utils.units import m2in, rad2Deg, deg2Rad, sign
import wpimath.controller
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedThroughBoreHexEncoder import WrapperedThroughBoreHexEncoder


#The most important question - why is this two separate things?? I think they should be combined
#the same button-type thing will be pressed to bring the algae manipulator up to angle, then start spinning it
class AlgaeWristControl(metaclass=Singleton):
    def __init__(self):
        #one important assumption we're making right now is that we don't need limits on the algae manipulator based on elevator height

        #motor and encoder
        self.wristMotor = WrapperedSparkMax(ALGAE_WRIST_CANID, "AlgaeWristMotor", True)
        self.algaeAbsEnc = WrapperedThroughBoreHexEncoder(port=ALGAE_ENC_PORT, name="AlgaeEncOffset", mountOffsetRad=deg2Rad(ALGAE_ANGLE_ABS_POS_ENC_OFFSET))

        #PID stuff calibrations
        self.kG = Calibration(name="Algae kG", default=0.1, units="V/cos(deg)")
        self.kS = Calibration(name="Algae kS", default=0.1, units="V")
        self.kV = Calibration(name="Algae kV", default=0.1, units="V/rps")
        self.kP = Calibration(name="Algae kP", default=0.1, units="V/degErr")

        #position calibrations... an angle in degrees. Assumingt 0 is horizontal, - is down, etc.  
        self.inPos = Calibration(name="Intake Position", default = 0, units="deg")
        self.stowPos = Calibration(name="Stow Position", default = -90, units="deg")
        self.reefPos = Calibration(name="Reef Position", default = -45, units="deg")

        
        #positions
        self.pos = AlgaeWristState.STOW
        self.actualPos = 0
        self.curPosCmdDeg = self.stowPos.get()

        self.desState = TrapezoidProfile.State(self.curPosCmdDeg,0)

    
        #set P gain on controller
        self.wristMotor.setPID(self.kP.get(), 0.0, 0.0)

        #Profiler
        self.maxV = Calibration(name="Algae Max Velocity", default=90.0, units="deg/sec")
        self.maxA = Calibration(name="Algae Max Acceleration", default=90.0, units="dec/sec2")
        self.profiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxV.get(),self.maxA.get()))
        self.curState = self.profiler.State()

        # Relative Encoder Offsets
        # Relative encoders always start at 0 at power-on
        # However, we may or may not have the mechanism at the "zero" position when we powered on
        # These variables store an offset which is calculated from the absolute sensors
        # to make sure the relative sensors inside the encoders accurately reflect
        # the actual position of the mechanism
        #the above named the relative encoder offset, the below calculates it
        self.initFromAbsoluteSensor()
        #set the degree details for your goals. For stow, intake off gorund, etc. 
        #Also, the absolute offset will be a constant that you can set here or just have in constants 

        addLog("Algae Wrist State",lambda:self.pos.value,"enum")
        addLog("Algae Profiled Angle", lambda: self.curState.position, "deg")
        addLog("Algae Actual Angle", lambda: self.actualPos, "deg")

    def setDesPos(self, desState : AlgaeWristState):
        #this is called in teleop periodic to set the desired pos of algae manipulator
        self.curPosCmdDeg = self.changePos(desState)
        self.pos = desState

    def getAbsAngleMeas(self):
        return rad2Deg(self.algaeAbsEnc.getAngleRad())

    def _motorRadToAngleRad(self, motorRev):
        #get angle of algae manipulator arm using the motor sensor
        return motorRev * 1/ALGAE_GEARBOX_GEAR_RATIO - self.relEncOffsetRad

    def getAngleRad(self):
        return self._motorRadToAngleRad(self.wristMotor.getMotorPositionRad())

    def _AngleRadToMotorRad(self,armAng):
        return ((armAng + self.relEncOffsetRad)* ALGAE_GEARBOX_GEAR_RATIO)

    # This routine uses the absolute sensors to adjust the offsets for the relative sensors
    # so that the relative sensors match reality.
    # It should be called.... infrequently. Likely once shortly after robot init.
    def initFromAbsoluteSensor(self):
        self.relEncOffsetRad = 0.0

        self.relEncOffsetRad = self.getAbsAngleMeas() - self.getAngleRad()

    # Might optimize to accept 1 enum parameter for new position
    def changePos(self,Pos):
        if(Pos == AlgaeWristState.INTAKEOFFGROUND):
            return self.inPos.get()
        elif(Pos == AlgaeWristState.REEF):
            return self.reefPos.get()
        else:
            return self.stowPos.get()

    def update(self):

        self.actualPos = self.getAbsAngleMeas()

        # Update profiler desired state based on any change in algae angle goal
        self.desState = TrapezoidProfile.State(self.curPosCmdDeg,0)

        # Update motor closed-loop calibration
        if(self.kP.isChanged()):
            self.wristMotor.setPID(self.kP.get(), 0.0, 0.0)

        self.curState = self.profiler.calculate(0.02, self.curState, self.desState)

        motorPosCmd = self._AngleRadToMotorRad(self.curState.position)
        motorVelCmd = self._AngleRadToMotorRad(self.curState.velocity)

        vFF = self.kV.get() * motorVelCmd  + self.kS.get() * sign(motorVelCmd) \
            + self.kG.get()

        self.wristMotor.setPosCmd(motorPosCmd, vFF)



class AlgeaIntakeControl(metaclass=Singleton):
    def __init__(self):
        #initialize all of your variables, spark maxes, etc. 
        self.intakeCommandState = False
        self.ejectCommandState = False
        self.algaeMotor = WrapperedSparkMax(ALGAE_INT_CANID, "AlgaeIntakeMotor", True)

        self.intakeVoltageCal = Calibration("IntakeVoltage", 12, "V")
        self.ejectVoltageCal = Calibration("EjectVoltage", -12, "V")
        self.holdVoltageCal = Calibration("HoldVoltage", 5, "V")

        self.hasGamePiece = False

        self.tofSensor = TimeOfFlight(ALGAE_GAMEPIECE_CANID)
        self.tofSensor.setRangingMode(TimeOfFlight.RangingMode.kShort, 24)
        self.tofSensor.setRangeOfInterest(6, 6, 10, 10)  # fov for sensor # game piece sensor empty for now
        #TODO: LOOK AT THIS LATER YOU SOFTWARE PEOPLE YOU BETTER LOOK AT THIS

        self.gamePiecePresentCal = Calibration("AlgaePresentThresh", 24, "in")

        self.disconTOFFault = faults.Fault("Algae Intake TOF Sensor is Disconnected")

        #addLog("Has Game Piece",lambda:self.hasGamePiece, "Bool")
        addLog("Algae intake",lambda:self.intakeCommandState,"Bool")
        addLog("Algae eject",lambda:self.ejectCommandState,"Bool")
        #addLog("Has Game Piece", self.getHasGamePiece, "Bool")

    
    def getHasGamePiece(self):
        gamepieceDistSensorMeas = m2in(self.tofSensor.getRange() / 1000.0)
        self.disconTOFFault.set(self.tofSensor.getFirmwareVersion() == 0)
        #update TOF (is it faulted? Is there a gamepiece being seen (see checkGamePiece function)?)
        if(self.disconTOFFault.isActive):
            # Gamepiece Sensor Faulted - assume we don't have a gamepiece
            return False
        else:
            if gamepieceDistSensorMeas < self.gamePiecePresentCal.get():
                return True
            else: 
                return False

    def update(self):
        self.hasGamePiece = self.getHasGamePiece()

        if self.intakeCommandState and not self.hasGamePiece:
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