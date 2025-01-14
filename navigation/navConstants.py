from wpimath.geometry import Pose2d, Rotation2d

"""
Constants related to navigation
"""


# Happy Constants for the goal poses we may want to drive to
GOAL_PICKUP = Pose2d.fromFeet(40,5,Rotation2d.fromDegrees(0.0))
GOAL_SPEAKER = Pose2d.fromFeet(3,20,Rotation2d.fromDegrees(180.0))

GOAL_1A = Pose2d(5.857,3.921,Rotation2d.fromDegrees(180.0))
GOAL_1B = Pose2d(5.870,4.189,Rotation2d.fromDegrees(180.0))
GOAL_2A = Pose2d(5.535,5.170,Rotation2d.fromDegrees(56.976 + 90 - 10))
GOAL_2B = Pose2d(5.139,5.412,Rotation2d.fromDegrees(56.976 + 90 - 10))
GOAL_3A = Pose2d(3.871,5.974,Rotation2d.fromDegrees(122.196 - 90 + 10))
GOAL_3B = Pose2d(3.600,5.331,Rotation2d.fromDegrees(122.196 - 90 + 10))
GOAL_4A = Pose2d(2.766,4.285,Rotation2d.fromDegrees(0.0))
GOAL_4B = Pose2d(2.766,3.874,Rotation2d.fromDegrees(0.0))
GOAL_5A = Pose2d(3.387,2.861,Rotation2d.fromDegrees(62.452 - 90 - 10))
GOAL_5B = Pose2d(3.896,2.532,Rotation2d.fromDegrees(62.452 - 90 - 10))
GOAL_6A = Pose2d(5.109,2.612,Rotation2d.fromDegrees(123.111 + 90))
GOAL_6B = Pose2d(5.460,2.706,Rotation2d.fromDegrees(123.111 + 90))