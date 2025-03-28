from enum import Enum
from utils.units import in2m

class AlgaeWristState(Enum):
    BARGE = 0
    STOW = 1
    REEF = 2
    NOTHING = 3
    PROCESSOR = 4
    INTAKEOFFGROUND = 5

# Enum for all four Level height commands for the elevator
class ElevatorLevelCmd(Enum):
    L1 = 0
    L2 = 1
    L3 = 2
    L4 = 3
    AL2 = 4
    AL3 = 5
    BARGE = 6
    NO_CMD = -1

class CoralManState(Enum):
    DISABLED = 0
    INTAKING = 1
    EJECTING = 2 
    HOLDING = 3

ALGAE_ANGLE_ABS_POS_ENC_OFFSET = 95.06
ALGAE_GEARBOX_GEAR_RATIO = 1#the max speed/acceleration the elevator can go

#All Numbers are placeholders for now (numbers used last year)
ELEV_GEARBOX_GEAR_RATIO = 16.0/1.0
ELEV_SPOOL_RADIUS_M = in2m(1.8)
MAX_ELEV_VEL_MPS = in2m(60)
MAX_ELEV_ACCEL_MPS2 = in2m(250)

