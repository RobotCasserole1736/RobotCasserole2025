from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from utils.constants import reefLocation
from wpimath.units import inchesToMeters
from drivetrain.drivetrainPhysical import WHEEL_BASE_HALF_LENGTH_M, BUMPER_THICKNESS_M


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

# Fudge Factors
# Nominally, these are all zero. Make them not-zero to tweak for specific score positions
# and account for field assembly questions.


# Rotations that we should be at when scoring
# Pulled from CAD model of field
# Must be in order from 1-6
SIDE_ROTS = [
Rotation2d.fromDegrees(180.0), # Side 1
Rotation2d.fromDegrees(240.0), # Side 2
Rotation2d.fromDegrees(300.0), # Side 3
Rotation2d.fromDegrees(0.0),   # Side 4
Rotation2d.fromDegrees(60.0),  # Side 5
Rotation2d.fromDegrees(120.0), # Side 6
]

# Radius from center of reef to center of the face
# Pulled from CAD model of field
REEF_RADIUS = inchesToMeters(32.1) 

# Distance from center of face, out to the reef score peg 
# Pulled from CAD model of field
SCORE_PEG_CENTER_DIST = inchesToMeters(6.48)

# Distance we want to be from the reef center while scoring
# Sum of reef size and robot size.
SCORE_DIST_FROM_REEF_CENTER = \
    REEF_RADIUS  + \
    WHEEL_BASE_HALF_LENGTH_M + \
    BUMPER_THICKNESS_M 

goalListTot = []
# Generate the set of a/b poses in order
for rot in SIDE_ROTS:
    # start at the reef location, pointed in the right direction.
    tmp = Pose2d(reefLocation, rot)
    # Transform to the nominal score locations
    aPose = tmp.transformBy(Transform2d(-1.0* SCORE_DIST_FROM_REEF_CENTER, SCORE_PEG_CENTER_DIST, Rotation2d()))
    bPose = tmp.transformBy(Transform2d(-1.0* SCORE_DIST_FROM_REEF_CENTER, -1.0* SCORE_PEG_CENTER_DIST, Rotation2d()))

    goalListTot.append(aPose)
    goalListTot.append(bPose)

