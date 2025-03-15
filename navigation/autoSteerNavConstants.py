from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from utils.constants import blueReefLocation, redReefLocation
from wpimath.units import inchesToMeters
from drivetrain.drivetrainPhysical import WHEEL_BASE_HALF_LENGTH_M, BUMPER_THICKNESS_M
from utils.allianceTransformUtils import onRed

"""
Constants related to navigation
"""
# Our convention for scoring position
#       Y
#       ^
#       |           3A     ^    2B        
# Blue1 |               /     \   
#       |        3B   /         \  2A
#    ---|           /             \  
#       |     4A   |               |  1B
# Blue2 |          |               |
#       |     4B   |               |  1A
#    ---|           \             / 
#       |        5A   \         /   6B
# Blue3 |               \     /
#       |            5B    v      6A
# <-----+-----------------------------------------> X
#       |
#       v

#####################################################################################
# Fudge Factors
# Theoretically, these are the only thing you tweak in this file
# Nominally, these are all zero. Make them not-zero to tweak for specific score positions
# and account for field assembly questions.

# Overall in-out fudge factor for RED Reef
FUDGE_DIST_IN_OUT_RED = [ # Negative is further from the reef, Positive is closer in.
inchesToMeters(0.0), # 1a
inchesToMeters(0.0), # 1b
inchesToMeters(0.0), # 2a
inchesToMeters(0.0), # 2b
inchesToMeters(0.0), # 3a
inchesToMeters(0.0), # 3b
inchesToMeters(0.0), # 4a
inchesToMeters(0.0), # 4b
inchesToMeters(0.0), # 5a
inchesToMeters(0.0), # 5b
inchesToMeters(0.0), # 6a
inchesToMeters(0.0), # 6b
]

# Left/Right fudge factor for RED Reef
FUDGE_DIST_LEFT_RIGHT_RED = [ # Positive is to the robot's left, negative is to the robot's right
inchesToMeters(0.0), # 1a
inchesToMeters(0.0), # 1b
inchesToMeters(0.0), # 2a
inchesToMeters(0.0), # 2b
inchesToMeters(0.0), # 3a
inchesToMeters(0.0), # 3b
inchesToMeters(0.0), # 4a
inchesToMeters(0.0), # 4b
inchesToMeters(0.0), # 5a
inchesToMeters(0.0), # 5b
inchesToMeters(0.0), # 6a
inchesToMeters(0.0), # 6b
]

# Overall in-out fudge factor for BLUE Reef
FUDGE_DIST_IN_OUT_BLUE = [ # Negative is further from the reef, Positive is closer in.
inchesToMeters(0.0), # 1a
inchesToMeters(0.0), # 1b
inchesToMeters(0.0), # 2a
inchesToMeters(0.0), # 2b
inchesToMeters(0.0), # 3a
inchesToMeters(0.0), # 3b
inchesToMeters(0.0), # 4a
inchesToMeters(0.0), # 4b
inchesToMeters(0.0), # 5a
inchesToMeters(0.0), # 5b
inchesToMeters(0.0), # 6a
inchesToMeters(0.0), # 6b
]

# Left/Right fudge factor for BLUE Reef
FUDGE_DIST_LEFT_RIGHT_BLUE = [ #Positive is to the robot's left, negative is to the robot's right
inchesToMeters(0.0), # 1a
inchesToMeters(0.0), # 1b
inchesToMeters(0.0), # 2a
inchesToMeters(0.0), # 2b
inchesToMeters(0.0), # 3a
inchesToMeters(0.0), # 3b
inchesToMeters(0.0), # 4a
inchesToMeters(0.0), # 4b
inchesToMeters(0.0), # 5a
inchesToMeters(0.0), # 5b
inchesToMeters(0.0), # 6a
inchesToMeters(0.0), # 6b
]

#####################################################################################

# Rotations that we should be at when scoring
# Pulled from CAD model of field
# Must be in order from 1A-6B
GOAL_ROTS = [
Rotation2d.fromDegrees(180.0), # 1a
Rotation2d.fromDegrees(180.0), # 1b
Rotation2d.fromDegrees(240.0), # 2a
Rotation2d.fromDegrees(240.0), # 2b
Rotation2d.fromDegrees(300.0), # 3a
Rotation2d.fromDegrees(300.0), # 3b
Rotation2d.fromDegrees(0.0),   # 4a
Rotation2d.fromDegrees(0.0),   # 4b
Rotation2d.fromDegrees(60.0),  # 5a
Rotation2d.fromDegrees(60.0),  # 5b
Rotation2d.fromDegrees(120.0), # 6a
Rotation2d.fromDegrees(120.0), # 6b
]

# Radius from center of reef to center of the face
# Pulled from CAD model of field
REEF_RADIUS = inchesToMeters(32.1) 

# Distance from center of face, out to the reef score peg 
# Pulled from CAD model of field
SCORE_PEG_CENTER_DIST = inchesToMeters(6.48)

#Arbitrary number for where we want to autosteer point towards
#how far (positive number) is the edge of the pink coral pipes from the reef edge
SCORE_POINT_DIST = inchesToMeters(0)

# Distance we want to be from the reef center while scoring
# Sum of reef size and robot size.
SCORE_DIST_FROM_REEF_CENTER = \
    REEF_RADIUS - SCORE_POINT_DIST

# Pre-calculate blue goals
_goalListCacheBlue = []
for idx, rot in enumerate(GOAL_ROTS):
    # start at the reef location, pointed in the right direction.
    tmp = Pose2d(blueReefLocation, rot)
    # Transform to the score locations

    # Nominal distance from center
    inOutDistance = -1.0 * SCORE_DIST_FROM_REEF_CENTER

    # Fudged
    inOutDistance += FUDGE_DIST_IN_OUT_BLUE[idx]

    # A is the "left hand" post from the robot's perspective
    # B is the "right hand" post from the robot's perspective
    # Even indicies (0,2,4,6) are A, odd are B
    leftRightOffset = (1.0 if (idx % 2 == 0) else -1.0) * SCORE_PEG_CENTER_DIST

    # Fudge the left/right distance
    leftRightOffset += FUDGE_DIST_LEFT_RIGHT_BLUE[idx]

    tmp = tmp.transformBy(Transform2d(inOutDistance, leftRightOffset, Rotation2d()))
    _goalListCacheBlue.append(tmp)


# Pre-calculate red goals
_goalListCacheRed = []
for idx, rot in enumerate(GOAL_ROTS):
    # start at the reef location, pointed in the right direction.
    rot = Rotation2d.fromDegrees(180) + rot # Invert for other side of the field
    tmp = Pose2d(redReefLocation, rot)
    # Transform to the score locations

    # Nominal distance from center
    inOutDistance = -1.0 * SCORE_DIST_FROM_REEF_CENTER

    # Fudged
    inOutDistance += FUDGE_DIST_IN_OUT_RED[idx]

    # A is the "left hand" post from the robot's perspective
    # B is the "right hand" post from the robot's perspective
    # Even indicies (0,2,4,6) are A, odd are B
    leftRightOffset = (1.0 if (idx % 2 == 0) else -1.0) * SCORE_PEG_CENTER_DIST

    # Fudge the left/right distance
    leftRightOffset += FUDGE_DIST_LEFT_RIGHT_RED[idx]

    tmp = tmp.transformBy(Transform2d(inOutDistance, leftRightOffset, Rotation2d()))
    _goalListCacheRed.append(tmp)



# NOTE - This function returns goals ALREADY transformed to the correct side.
# You do NOT need to call transform again on the result of poses
def getTransformedGoalList() -> list[Pose2d]:
    if(onRed()):
        return _goalListCacheRed
    else:
        return _goalListCacheBlue




