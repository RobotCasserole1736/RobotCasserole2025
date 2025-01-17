from numpy import array
from wpimath.geometry import Pose2d, Rotation2d

"""
Constants related to navigation
"""


# Happy Constants for the goal poses we may want to drive to
GOAL_PICKUP = Pose2d.fromFeet(40,5,Rotation2d.fromDegrees(0.0))
GOAL_SPEAKER = Pose2d.fromFeet(3,20,Rotation2d.fromDegrees(180.0))

GOAL_1A = Pose2d(5.954,3.815,Rotation2d.fromDegrees(180.0))
GOAL_1B = Pose2d(5.954,4.193,Rotation2d.fromDegrees(180.0))
GOAL_2A = Pose2d(5.370,5.215,Rotation2d.fromDegrees(240.0))
GOAL_2B = Pose2d(5.080,5.383,Rotation2d.fromDegrees(240.0))
GOAL_3A = Pose2d(3.871,5.974,Rotation2d.fromDegrees(300.0))
GOAL_3B = Pose2d(3.600,5.331,Rotation2d.fromDegrees(300.0))
GOAL_4A = Pose2d(2.766,4.285,Rotation2d.fromDegrees(0.0))
GOAL_4B = Pose2d(2.766,3.874,Rotation2d.fromDegrees(0.0))
GOAL_5A = Pose2d(3.387,2.861,Rotation2d.fromDegrees(60.0))
GOAL_5B = Pose2d(3.896,2.532,Rotation2d.fromDegrees(60.0))
GOAL_6A = Pose2d(5.109,2.612,Rotation2d.fromDegrees(120.0))
GOAL_6B = Pose2d(5.460,2.706,Rotation2d.fromDegrees(120.0))

goalListTot = [GOAL_1A, GOAL_1B, GOAL_2A, GOAL_2B, GOAL_3A, GOAL_3B, GOAL_4A, GOAL_4B, GOAL_5A, GOAL_5B, GOAL_6A, GOAL_6B]
