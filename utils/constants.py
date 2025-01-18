# Constants we may need
# Just starting with the minimum stuff we need
# The math conversions are under units.py

#######################################################################################
## FIELD DIMENSIONS
#######################################################################################

FIELD_X_M = 17.548 # "Length"
FIELD_Y_M = 8.062  # "Width"

#######################################################################################
## CAN ID'S
#######################################################################################

# Reserved_CANID = 0
# Reserved_CANID = 1
DT_FR_WHEEL_CANID = 19
DT_FR_AZMTH_CANID = 8
DT_FL_WHEEL_CANID = 28
DT_FL_AZMTH_CANID = 3
DT_BR_WHEEL_CANID = 49
DT_BR_AZMTH_CANID = 5
DT_BL_AZMTH_CANID = 18
DT_BL_WHEEL_CANID = 53
ELEV_RM_CANID = 10
ELEV_LM_CANID = 11
ELEV_TOF_CANID = 12
# change ELEV CANID later
# Unused_CANID = 13
# Unused_CANID = 14
# Unused_CANID = 15
# Unused_CANID = 16




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
# Unused = 7
# Unused = 8
LED_STACK_LIGHT_CTRL_PWM = 9


#######################################################################################
## DIO Bank
#######################################################################################

DT_BR_AZMTH_ENC_PORT = 0
DT_FL_AZMTH_ENC_PORT = 1
DT_BL_AZMTH_ENC_PORT = 2
DT_FR_AZMTH_ENC_PORT = 3
# Unused = 4
# Unused = 5
# Unused = 6
# Unused = 7
FIX_ME_LED_PIN = 8
HEARTBEAT_LED_PIN = 9
