from playingwithfusion import TimeOfFlight
from Elevatorandmech.ElevatorandMechConstants import ELEV_HEIGHT, MAX_ELEV_ACCEL_MPS2, MAX_ELEV_VEL_MPS, ELEV_GEARBOX_GEAR_RATIO, ELEV_SPOOL_RADIUS_M, ElevatorLevelCmd
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import addLog, log
from utils.constants import ELEV_LM_CANID, ELEV_RM_CANID, ELEV_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wpimath.trajectory import TrapezoidProfile

REEF_L1_HEIGHT_M = 0.5842
REEF_L2_HEIGHT_M = 0.9398
REEF_L3_HEIGHT_M = 1.397 
REEF_L4_HEIGHT_M = 2.159 
ELEV_MIN_HEIGHT_M = REEF_L1_HEIGHT_M  # TODO - is elevator's bottom position actually L1?

class ElevatorControl(metaclass=Singleton):
    def __init__(self):

        # Coral Scoring Heights in meters
        self.L1_Height = Calibration(name="Elevator Preset Height L1", units="m", default=REEF_L1_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L2_Height = Calibration(name="Elevator Preset Height L2", units="m", default=REEF_L2_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L3_Height = Calibration(name="Elevator Preset Height L3", units="m", default=REEF_L3_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L4_Height = Calibration(name="Elevator Preset Height L4", units="m", default=REEF_L4_HEIGHT_M - ELEV_MIN_HEIGHT_M)

        self.manAdjMaxVoltage = Calibration(name="Elevator Manual Adj Max Voltage", default=1.0, units="V")

        self.curHeightGoal = ElevatorLevelCmd.NO_CMD
        self.heightGoal = self.L1_Height.get()
        self.coralSafe = True
        self.manualAdjCmd = 0.0

        self.desState = TrapezoidProfile.State(self.heightGoal,0)

        # Elevator Motors
        self.Rmotor = WrapperedSparkMax(ELEV_RM_CANID, "ElevatorMotorRight", brakeMode=True)
        self.LMotor = WrapperedSparkMax(ELEV_LM_CANID, "ElevatorMotorLeft", brakeMode=True)
        #we don't know if we want to invert LMotor (left) or not when we follow RMotor (right), automatically assumed False
        self.LMotor.setFollow(ELEV_RM_CANID)

        # FF and proportional gain constants
        self.kV = Calibration(name="Elevator kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.25, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.05, units="V/rad error")

        # Set P gain on motor
        self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        # Profiler
        self.maxV = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_MPS, units="mps")
        self.maxA = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_MPS2, units="mps2")
        self.profiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxV.get(),self.maxA.get()))
        self.curState = self.profiler.State()

        self.actualPos = 0
        self.stopped = False

        # Limit switch code; bottom for resetting offset and ensuring it starts correctly, top for saftey to stop from spinning
        # Only for protection, not for initializing the elevator height
        # TODO - implement limit switches here

        # Playing with Fusion time of flight sensor for initalizing elevator height
        self.heightAbsSen = TimeOfFlight(ELEV_TOF_CANID)

        # Absolute Sensor mount offsets
        # After mounting the sensor, these should be tweaked one time
        # in order to adjust whatever the sensor reads into the reference frame
        # of the mechanism
        self.ABS_SENSOR_READING_AT_ELEVATOR_BOTTOM_M = 0.074 # TODO correct?

        # Relative Encoder Offsets
        # Releative encoders always start at 0 at power-on
        # However, we may or may not have the mechanism at the "zero" position when we powered on
        # These variables store an offset which is calculated from the absolute sensors
        # to make sure the relative sensors inside the encoders accurately reflect
        # the actual position of the mechanism
        self.relEncOffsetM = 0.0
        # Create a motion profile with the given maximum velocity and maximum
        # acceleration constraints for the next setpoint.

        # Add some helpful log values
        addLog("Elevator Actual Height", lambda: self.actualPos, "m")
        addLog("Elevator Goal Height", lambda: self.heightGoal, "m")
        addLog("Elevator Stopped", lambda: self.stopped, "bool")
        addLog("Elevator Profiled Height", lambda: self.curState.position, "m")

        # Finally, one-time init the relative sensor offsets from the absolute sensors
        self.initFromAbsoluteSensor()

        # TODO - limit switch config?

    def _RmotorRadToHeight(self, RmotorRad):
        return RmotorRad * 1/ELEV_GEARBOX_GEAR_RATIO * (ELEV_SPOOL_RADIUS_M) - self.relEncOffsetM
    
    def _heightToMotorRad(self, elevLin):
        return ((elevLin + self.relEncOffsetM)*1/(ELEV_SPOOL_RADIUS_M) * ELEV_GEARBOX_GEAR_RATIO)
    
    def _heightVeltoMotorVel(self, elevLinVel):
        return (elevLinVel *1/(ELEV_SPOOL_RADIUS_M) * ELEV_GEARBOX_GEAR_RATIO)
    
    def getHeightM(self):
        return self._RmotorRadToHeight(self.Rmotor.getMotorPositionRad()) 
    
    #return the height of the elevator as measured by the absolute sensor in meters
    def _getAbsHeight(self):
        return self.heightAbsSen.getRange() / 1000.0 - self.ABS_SENSOR_READING_AT_ELEVATOR_BOTTOM_M

    # This routine uses the absolute sensors to adjust the offsets for the relative sensors
    # so that the relative sensors match reality.
    # It should be called.... infrequently. Likely once shortly after robot init.
    def initFromAbsoluteSensor(self):
        # Reset offsets to zero, so the relative sensor get functions return
        # just whatever offset the relative sensor currently has.
        self.relEncOffsetM = 0.0

        # New Offset = real height - what height says?? 
        self.relEncOffsetM = self._getAbsHeight() - self.getHeightM()

    def update(self):
        self.actualPos = self.getHeightM()


        if self.coralSafe:
            self.stopped = (self.curHeightGoal == ElevatorLevelCmd.NO_CMD)

            if self.curHeightGoal == ElevatorLevelCmd.L1:
                self.heightGoal = self.L1_Height.get()
            elif self.curHeightGoal == ElevatorLevelCmd.L2:
                self.heightGoal = self.L2_Height.get()
            elif self.curHeightGoal == ElevatorLevelCmd.L3:
                self.heightGoal = self.L3_Height.get()
            elif self.curHeightGoal == ElevatorLevelCmd.L4:
                self.heightGoal = self.L4_Height.get()
            # Else, no change to height goal
        else:
            # Coral blocks motion, must get it out of the way first.
            self.stopped=True

        # Update profiler desired state based on any change in height goal
        self.desState = TrapezoidProfile.State(self.heightGoal,0)


        # Update motor closed-loop calibration
        if(self.kP.isChanged()):
            self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        if(self.stopped):
            # Handle stopped by just holding mechanism in place with gravity offset, no closed loop.
            # TODO - do we need a more gentle stop here?
            manAdjVoltage = self.manAdjMaxVoltage.get() * self.manualAdjCmd

            self.Rmotor.setVoltage(self.kG.get() + manAdjVoltage)
            self.curState = TrapezoidProfile.State(self.actualPos,0)
        else:
            self.curState = self.profiler.calculate(0.02, self.curState, self.desState)

            motorPosCmd = self._heightToMotorRad(self.curState.position)
            motorVelCmd = self._heightVeltoMotorVel(self.curState.velocity)

            vFF = self.kV.get() * motorVelCmd  + self.kS.get() * sign(motorVelCmd) \
                + self.kG.get()

            self.Rmotor.setPosCmd(motorPosCmd, vFF)

    # API to set current height goal
    def setHeightGoal(self, presetHeightCmd:ElevatorLevelCmd) -> None:
        self.curHeightGoal = presetHeightCmd

    # API to confirm we are oK to be at a height other than L1
    def setSafeToLeaveL1(self, safe:bool) -> None:
        self.coralSafe = safe

    def setManualAdjCmd(self, cmd:float) -> None:
        self.manualAdjCmd = cmd