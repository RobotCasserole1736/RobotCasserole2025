from enum import Enum


class AlgaeWristState(Enum):
    DISABLED = 0
    INTAKEOFFGROUND = 1
    STOW = 2
    REEF = 3

ALGAE_ANGLE_ABS_POS_ENC_OFFSET = 0
ALGAE_GEARBOX_GEAR_RATIO = 1