from typing import overload
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from choreo.trajectory import SwerveSample
from utils.constants import FIELD_X_M, FIELD_Y_M

"""
 Utilities to help flip from blue alliance to red if needed
 We chose a coordinate system where the origin is always in the 
 bottom left on the blue alliance.

 Note that this assumes the "mirroring" that happened in 2023 and 2024.
 A diagonally symmetric field (like 2020) would not need this.
"""

_flipToRight = False

def _shouldFlipToRight() -> bool:
    return _flipToRight and (wpilib.DriverStation.isAutonomous() or wpilib.DriverStation.isDisabled())

# Sets whether we are flipped to the right or not
def setFlip(toRight: bool):
    global _flipToRight
    _flipToRight = toRight

# Base flip for X axis to flip to the other side of the field.
def flipX(xIn):
    return xIn
    
# Base flip for X axis to flip to the other side of the field.
def flipY(yIn):
    if _flipToRight:
        return FIELD_Y_M - yIn
    else:
        return yIn


# Note that Y axis does not need any flipping

# Other types are flipped in the flip function

# The following typehints remove vscode errors by telling the linter
# exactly how the flip() function handles different types
@overload
def flip(valIn: None) -> None:
    pass

@overload
def flip(valIn: Rotation2d) -> Rotation2d:
    pass

@overload
def flip(valIn: Translation2d) -> Translation2d:
    pass

@overload
def flip(valIn: Pose2d) -> Pose2d:
    pass

@overload
def flip(valIn: SwerveSample) -> SwerveSample:
    pass


# Actual implementation of the flip function
def flip(valIn):
    if(valIn is None):
        return None
    
    elif isinstance(valIn, Rotation2d):
        if _shouldFlipToRight():
            return Rotation2d.fromDegrees(0) - valIn
        else:
            return valIn

    elif isinstance(valIn, Translation2d):
        if _shouldFlipToRight():
            return Translation2d(flipX(valIn.X()), flipY(valIn.Y()))
        else:
            return valIn

    elif isinstance(valIn, Transform2d):
        if _shouldFlipToRight():
            trans = flip(valIn.translation())
            rot = flip(valIn.rotation())
            return Transform2d(trans, rot)
        else:
            return valIn

    elif isinstance(valIn, Pose2d):
        if _shouldFlipToRight():
            trans = flip(valIn.translation())
            rot = flip(valIn.rotation())
            return Pose2d(trans, rot)
        else:
            return valIn

    elif isinstance(valIn, SwerveSample):
        if _shouldFlipToRight():
            return SwerveSample(
                valIn.timestamp,
                flipX(valIn.x),
                flipY(valIn.y),
                -1.0 * valIn.heading,
                valIn.vx,
                -1.0 * valIn.vy,
                -1.0 * valIn.omega,
                valIn.ax,
                -1.0 * valIn.ay,
                -1.0 * valIn.alpha,
                [],[]
            )
        else:
            return valIn

    else:
        raise TypeError(f"flip function received unknown type:{type(valIn).__name__}")
