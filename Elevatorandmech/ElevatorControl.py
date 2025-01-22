from playingwithfusion import TimeOfFlight
from Elevatorandmech.ElevatorandMechConstants import MAX_ELEV_ACCEL_MPS2, MAX_ELEV_VEL_MPS, ELEV_GEARBOX_GEAR_RATIO, ELEV_SPOOL_RADIUS_M
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import log
from utils.constants import ELEV_LM_CANID, ELEV_RM_CANID, ELEV_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from rev import SparkBase, SparkBaseConfig, SparkMax, SparkLowLevel, SparkMaxConfig
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer
#from Elevatorandmech.Profiler import ProfiledAxis

# need to figure out a way to import
class ElevatorControl():
    def __init__(self):
        self.Rmotor = WrapperedSparkMax(ELEV_RM_CANID, "ElevatorMotor", brakeMode=True)
        #self.Rmotor.setInverted(True)
        self.LMotor = WrapperedSparkMax(ELEV_LM_CANID, SparkLowLevel.MotorType.kBrushless, brakeMode=True)
        self.LMotor.setFollow(ELEV_RM_CANID)
        #we don't know if we want to invert LMotor (left) or not when we follow RMotor (right), automatically assumed False
        self.maxV = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_MPS, units="mps")
        self.maxA = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_MPS2, units="mps2")
        
        self.motorVelCmd = 0.0

        self.profiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxV.get(),self.maxA.get()))
        
        self.heightAbsSen = TimeOfFlight(ELEV_TOF_CANID)
        self.heightAbsSen.setRangingMode(TimeOfFlight.RangingMode.kShort, 24)
        self.heightAbsSen.setRangeOfInterest(6, 6, 10, 10)  # fov for sensor

        self.kV = Calibration(name="Elevator kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.25, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.05, units="V/rad error")

        self.Rmotor.setPID(self.kV.get(), 0.0, 0.0)
        self.Rmotor.setPID(self.kS.get(), 0.0, 0.0)
        self.Rmotor.setPID(self.kG.get(), 0.0, 0.0)
        self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        self.stopped = True
        # Absolute Sensor mount offsets
        # After mounting the sensor, these should be tweaked one time
        # in order to adjust whatever the sensor reads into the reference frame
        # of the mechanism
        self.absOffsetM = 0.074

        # Relative Encoder Offsets
        # Releative encoders always start at 0 at power-on
        # However, we may or may not have the mechanism at the "zero" position when we powered on
        # These variables store an offset which is calculated from the absolute sensors
        # to make sure the relative sensors inside the encoders accurately reflect
        # the actual position of the mechanism
        self.relEncOffsetM = 0.0
        # Create a motion profile with the given maximum velocity and maximum
        # acceleration constraints for the next setpoint.
        self.profile = TrapezoidProfile(TrapezoidProfile.Constraints(1.75, 0.75))

        self.goal = TrapezoidProfile.State()
        self.setpoint = TrapezoidProfile.State()

        self.profiledPos = 0.0
        self.curUnprofiledPosCmd = 0.0
    def _RmotorRadToHeight(self, RmotorRad):
        return RmotorRad * 1/ELEV_GEARBOX_GEAR_RATIO * (ELEV_SPOOL_RADIUS_M) - self.relEncOffsetM
    def _heightToMotorRad(self, elevLin):
        return ((elevLin + self.relEncOffsetM)*1/(ELEV_SPOOL_RADIUS_M) * ELEV_GEARBOX_GEAR_RATIO)
    def _heightVeltoMotorVel(self, elevLinVel);
        return (elevLinVel *1/(ELEV_SPOOL_RADIUS_M) * ELEV_GEARBOX_GEAR_RATIO)
    def getHeightM(self):
        return self._RmotorRadToHeight(self.Rmotor.getMotorPositionRad())
    #def update(self):  
        #update current sensor heights, etc. 
        #pass
    def update(self):
        actualPos = self.getHeightM()

        # Update motor closed-loop calibration
        if(self.kP.isChanged()):
            self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        if(self.stopped):
            self.Rmotor.setVoltage(0.0)
            self.profiledPos = actualPos
        else:
            curState = self.profiler.State()
            desState = TrapezoidProfile.State(0,0)
            self.profiler.calculate(0.02,curState,desState)

            self.profiledPos = curState.position

            motorPosCmd = self._heightToMotorRad(curState.position)
            self.motorVelCmd = self._heightVeltoMotorVel(curState.velocity)

            vFF = self.kV.get() * self.motorVelCmd  + self.kS.get() * sign(self.motorVelCmd) + self.kG.get()

            self.Rmotor.setPosCmd(motorPosCmd, vFF)
            self.LMotor.setFollow(self.Rmotor.ctrl, True)

    def setHeightGoal(self, goal):
        #self.heightGoal = goal
        """
        if goal == heightStates.L1:
            self.heightGoal = L1_HEIGHT
        elif _____:
            ...
        
        """
        pass