#the max speed/acceleration the elevator can go
from enum import Enum
from utils.calibration import Calibration
from utils.units import in2m

#All Numbers are placeholders for now (numbers used last year)
ELEV_GEARBOX_GEAR_RATIO = 16.0/1.0
ELEV_SPOOL_RADIUS_M = in2m(1.8)
MAX_ELEV_VEL_MPS = in2m(24.0)
MAX_ELEV_ACCEL_MPS2 = in2m(24.0)
ELEV_HEIGHT = in2m(24 )
#elev height has to be changed

# Enum for all four Level height commands for the elevator
class ElevatorLevelCmd(Enum):
    L1 = 0
    L2 = 1
    L3 = 2
    L4 = 3
    NO_CMD = -1

