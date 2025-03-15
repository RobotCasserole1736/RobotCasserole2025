from Elevatorandmech.ElevatorandMechConstants import MAX_ELEV_ACCEL_MPS2, MAX_ELEV_VEL_MPS, ELEV_GEARBOX_GEAR_RATIO, ELEV_SPOOL_RADIUS_M, ElevatorLevelCmd
from utils.calibration import Calibration
from utils.mapLookup2d import MapLookup2D
from utils.units import sign
from utils.signalLogging import addLog
from utils.constants import ELEV_LM_CANID, ELEV_RM_CANID, ELEV_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wpimath.trajectory import TrapezoidProfile

REEF_L1_HEIGHT_M = 0.5842
REEF_L2_HEIGHT_M = 0.9398
REEF_L3_HEIGHT_M = 1.397 
REEF_L4_HEIGHT_M = 1.6
ELEV_MIN_HEIGHT_M = REEF_L1_HEIGHT_M  # TODO - is elevator's bottom position actually L1?

# The interfence zone exists between the heights where the coral makes contact with the elevator while
# not fully into the intake. To tune these:
# 1. move the elevator to the top
# 2. put coral sticking out the back of the intake
# 3. Slowly lower the elevator until the coral makes contact, note the height. This is the TOP
# 4. Remove the coral, move to the bottom.
# 5. raise the elevator slowly until the coral makes contact, note the height. This is the BOTTOM
ELEV_CORAL_INTERFERENCE_ZONE_TOP_M = .9
ELEV_CORAL_INTERFERENCE_ZONE_BOTTOM_M = 0.05
ELEV_CORAL_INTERFERENCE_ZONE_CENTER_M = (ELEV_CORAL_INTERFERENCE_ZONE_TOP_M + ELEV_CORAL_INTERFERENCE_ZONE_BOTTOM_M) / 2.0

class ElevatorControl(metaclass=Singleton):
    def __init__(self):

        # Coral Scoring Heights in meters
        self.L1_Height = Calibration(name="Elevator Preset Height L1", units="m", default=0.0)
        self.L2_Height = Calibration(name="Elevator Preset Height L2", units="m", default=.1847)
        self.L3_Height = Calibration(name="Elevator Preset Height L3", units="m", default=.6139)
        self.L4_Height = Calibration(name="Elevator Preset Height L4", units="m", default=1.25)
        self.AL2_Height = Calibration(name="Elevator Preset Height Algae L2", units="m", default=.638)
        self.AL3_Height = Calibration(name="Elevator Preset Height Algae L3", units="m", default=1.075)


        self.manAdjMaxVoltage = Calibration(name="Elevator Manual Adj Max Voltage", default=3.0, units="V")

        self.curHeightGoal = ElevatorLevelCmd.NO_CMD
        self.heightGoal = self.L1_Height.get()
        self.coralSafe = True
        self.manualAdjCmd = 0.0

        self.desState = TrapezoidProfile.State(self.heightGoal,0)

        # Elevator Motors
        self.Rmotor = WrapperedSparkMax(ELEV_RM_CANID, "ElevatorMotorRight", brakeMode=True)
        self.LMotor = WrapperedSparkMax(ELEV_LM_CANID, "ElevatorMotorLeft", brakeMode=True)
        #we don't know if we want to invert LMotor (left) or not when we follow RMotor (right), automatically assumed False
        self.LMotor.setInverted(False)
        self.Rmotor.setInverted(True)

        # FF and proportional gain constants
        self.kV = Calibration(name="Elevator kV", default=0.015, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.5, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.1, units="V/rad error")

        # Set P gain on motor
        self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)
        self.LMotor.setPID(self.kP.get(), 0.0, 0.0)

        # Profiler
        self.profiler = TrapezoidProfile(TrapezoidProfile.Constraints(MAX_ELEV_VEL_MPS,MAX_ELEV_ACCEL_MPS2))
        self.curState = self.profiler.State()

        self.actualPos = 0
        self.stopped = False

        # Relative Encoder Offsets
        # Releative encoders always start at 0 at power-on
        # However, we may or may not have the mechanism at the "zero" position when we powered on
        # These variables store an offset which is calculated from the absolute sensors
        # to make sure the relative sensors inside the encoders accurately reflect
        # the actual position of the mechanism
        self.relEncOffsetM = 0.0
        # Create a motion profile with the given maximum velocity and maximum
        # acceleration constraints for the next setpoint.

        # Drivetrain limit factor
        # Based on elevator height, limit the max speed of the drivetrain
        # Limits the drivetrain speed by a factor depending on elevator height
        self.dtSpeedLimitMap = MapLookup2D([
            # (elevator measured height, Dt Limit fraction)
            (0.0, 1.0),
            (1.0, 0.15) # TODO - tweak and tune as needed, these were randomly chosen by Chris
        ])

        # Add some helpful log values
        addLog("Elevator Actual Height", lambda: self.actualPos, "m")
        addLog("Elevator Goal Height", lambda: self.heightGoal, "m")
        addLog("Elevator Stopped", lambda: self.stopped, "bool")
        addLog("Elevator Profiled Height", lambda: self.curState.position, "m")

        # Finally, one-time init the relative sensor offsets from the absolute sensors
        self.zeroElevatorReading()

    def _RmotorRadToHeight(self, RmotorRad: float) -> float:
        return RmotorRad * 1/ELEV_GEARBOX_GEAR_RATIO * (ELEV_SPOOL_RADIUS_M) + self.relEncOffsetM
    
    def _heightToMotorRad(self, elevLin: float) -> float:
        return ((elevLin + self.relEncOffsetM)*1/(ELEV_SPOOL_RADIUS_M) * ELEV_GEARBOX_GEAR_RATIO)
    
    def _heightVeltoMotorVel(self, elevLinVel: float) -> float:
        return (elevLinVel *1/(ELEV_SPOOL_RADIUS_M) * ELEV_GEARBOX_GEAR_RATIO)
    
    def getHeightM(self) -> float:
        return (self._RmotorRadToHeight(self.Rmotor.getMotorPositionRad()))
    
    def atHeight(self): 
        return self.atElevHeight

    # Call this to zero out
    def zeroElevatorReading(self) -> None:
        # Reset offsets to zero, so the relative sensor get functions return
        # just whatever offset the relative sensor currently has.
        self.relEncOffsetM = 0.0

        # New Offset = real height - what height says?? 
        self.relEncOffsetM = -self.getHeightM()

    def update(self) -> None:
        self.actualPos = self.getHeightM()

        # Assign nominal goals
        self.stopped = (self.curHeightGoal == ElevatorLevelCmd.NO_CMD)
        if self.curHeightGoal == ElevatorLevelCmd.L1:
            self.heightGoal = self.L1_Height.get()
        elif self.curHeightGoal == ElevatorLevelCmd.L2:
            self.heightGoal = self.L2_Height.get()
        elif self.curHeightGoal == ElevatorLevelCmd.L3:
            self.heightGoal = self.L3_Height.get()
        elif self.curHeightGoal == ElevatorLevelCmd.L4:
            self.heightGoal = self.L4_Height.get()
        elif self.curHeightGoal == ElevatorLevelCmd.AL2:
            self.heightGoal = self.AL2_Height.get()
        elif self.curHeightGoal == ElevatorLevelCmd.AL3:
            self.heightGoal = self.AL3_Height.get()
           
        if not self.coralSafe:
            # Coral blocks motion. Modify goals/commands as needed.

            # Limit profiled motion
            if self.actualPos > ELEV_CORAL_INTERFERENCE_ZONE_CENTER_M:
                # We're currently on the "top" of the range of travel
                # Limit to have goals no lower than the top of the interference zone
                self.heightGoal = max(self.heightGoal, ELEV_CORAL_INTERFERENCE_ZONE_TOP_M) 
            elif self.actualPos <= ELEV_CORAL_INTERFERENCE_ZONE_CENTER_M:
                # We're currently on the "bottom" of the range of travel
                self.heightGoal = min(self.heightGoal, ELEV_CORAL_INTERFERENCE_ZONE_BOTTOM_M)

            # Limit manual command to not go into interference zone
            if ELEV_CORAL_INTERFERENCE_ZONE_TOP_M > self.actualPos > ELEV_CORAL_INTERFERENCE_ZONE_CENTER_M:
                # We're currently in the top-half of the interference zone
                self.manualAdjCmd = max(self.manualAdjCmd, 0) # only allow positive/up commands
            elif ELEV_CORAL_INTERFERENCE_ZONE_CENTER_M > self.actualPos > ELEV_CORAL_INTERFERENCE_ZONE_BOTTOM_M:
                # We're currently in the bottom-half of the interference zone
                self.manualAdjCmd = min(self.manualAdjCmd, 0) # only allow negative/down commands

        # Update profiler desired state based on any change in height goal
        self.desState = TrapezoidProfile.State(self.heightGoal,0)


        # Update motor closed-loop calibration
        if(self.kP.isChanged()):
            self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)
            self.LMotor.setPID(self.kP.get(), 0.0, 0.0)

        if(self.stopped):
            # Handle stopped by just holding mechanism in place with gravity offset, no closed loop.
            # TODO - do we need a more gentle stop here?
            manAdjVoltage = self.manAdjMaxVoltage.get() * self.manualAdjCmd

            self.Rmotor.setVoltage(self.kG.get() + manAdjVoltage)
            self.LMotor.setVoltage(self.kG.get() + manAdjVoltage)
            self.curState = TrapezoidProfile.State(self.actualPos,0)
        else:
            self.curState = self.profiler.calculate(0.04, self.curState, self.desState)

            motorPosCmd = self._heightToMotorRad(self.curState.position)
            motorActPos = self._heightToMotorRad(self.actualPos)
            motorVelCmd = self._heightVeltoMotorVel(self.curState.velocity)

            vFF = self.kV.get() * motorVelCmd  + self.kS.get() * sign(motorVelCmd) \
                + self.kG.get()
            
            vFB = self.kP.get() * (motorPosCmd - motorActPos)

            self.LMotor.setVoltage(vFF + vFB)
            self.Rmotor.setVoltage(vFF + vFB)

        if abs(self.heightGoal - self.actualPos) < .01:
            self.atElevHeight = True
        else: 
            self.atElevHeight = False


    # API to set current height goal
    def setHeightGoal(self, presetHeightCmd:ElevatorLevelCmd) -> None:
        self.curHeightGoal = presetHeightCmd

    # API to confirm we are oK to be at a height other than L1
    def setSafeToLeaveL1(self, safe:bool) -> None:
        self.coralSafe = safe

    def setManualAdjCmd(self, cmd:float) -> None:
        self.manualAdjCmd = cmd
    # Returns a limit factor to apply to drivetrain speed commands based on elevator height
    def getDtSpeedLimitFactor(self) -> float:
        return self.dtSpeedLimitMap.lookup(self.actualPos)