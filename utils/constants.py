# Constants we may need
# Just starting with the minimum stuff we need
# The math conversions are under units.py

from wpimath.geometry import Translation2d

#######################################################################################
## FIELD DIMENSIONS
#######################################################################################

FIELD_X_M = 17.548 # "Length"
FIELD_Y_M = 8.062  # "Width"

# Blue Reef Location
_REEF_LOC_X_M = 4.502
_REEF_LOC_Y_M = 4.0363
blueReefLocation = Translation2d(_REEF_LOC_X_M, _REEF_LOC_Y_M)
redReefLocation = Translation2d(FIELD_X_M - _REEF_LOC_X_M, _REEF_LOC_Y_M)

#######################################################################################
## CAN ID'S
#######################################################################################
# Reserved_CANID = 0
CORAL_IN_CANID = 21
DT_FR_WHEEL_CANID = 2
DT_FR_AZMTH_CANID = 3
DT_FL_WHEEL_CANID = 4
DT_FL_AZMTH_CANID = 5
DT_BR_WHEEL_CANID = 6
DT_BR_AZMTH_CANID = 7
DT_BL_WHEEL_CANID = 8
DT_BL_AZMTH_CANID = 9
ELEV_RM_CANID = 10
ELEV_LM_CANID = 11
ELEV_TOF_CANID = 12
CORAL_R_CANID = 13
CORAL_L_CANID = 14
ALGAE_INT_CANID = 15
ALGAE_WRIST_CANID = 16
CLIMB_CANID = 19
ALGAE_GAMEPIECE_CANID = 20

#######################################################################################
## PWM Bank
#######################################################################################

# Unused = 0
# Unused = 1
# Unused = 2
# Unused = 3
# Unused = 4
# Unused = 5
# Unused = 6
# Unused = 8
# Unused = 7
LED_STACK_LIGHT_CTRL_PWM = 9


#######################################################################################
## DIO Bank
#######################################################################################

DT_FR_AZMTH_ENC_PORT = 0
DT_BR_AZMTH_ENC_PORT = 1
DT_BL_AZMTH_ENC_PORT = 2
DT_FL_AZMTH_ENC_PORT = 3
CORAL_GAME_PIECE_B_PORT = 4
CORAL_GAME_PIECE_F_PORT = 5
ELEV_TOF_CANID = 6
HEARTBEAT_LED_PIN = 7
FIX_ME_LED_PIN = 8
ALGAE_ENC_PORT = 9
