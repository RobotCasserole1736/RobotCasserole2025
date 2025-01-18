from playingwithfusion import TimeOfFlight
from Elevatorandmech.ElevatorandMechConstants import MAX_ELEV_ACCEL_MPS2, MAX_ELEV_VEL_MPS
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import log
from utils.constants import ELEV_LM_CANID, ELEV_RM_CANID, ELEV_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from rev import SparkBase, SparkBaseConfig, SparkMax, SparkLowLevel, SparkMaxConfig

# need to figure out a way to import
class ElevatorControl():
    def __init__(self):
        pass
        self.Rmotor = WrapperedSparkMax(ELEV_RM_CANID, "ElevatorMotor", brakeMode=True)
        #self.Rmotor.setInverted(True)
        self.LMotor = WrapperedSparkMax(ELEV_LM_CANID, SparkLowLevel.MotorType.kBrushless, brakeMode=True)
        self.LMotor.setFollow(ELEV_RM_CANID)
        #we don't know if we want to invert LMotor (left) or not when we follow RMotor (right), automatically assumed False
        self.maxV = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_MPS, units="mps")
        self.maxA = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_MPS2, units="mps2")
        
        self.motorVelCmd = 0.0

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

    def update(self):  
        pass