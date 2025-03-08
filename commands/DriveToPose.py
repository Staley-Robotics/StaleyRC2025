import typing
import math

from pathplannerlib.path import GoalEndState
from wpilib import SmartDashboard

from wpimath import applyDeadband
from wpimath.geometry import Pose2d
from wpimath.units import degrees, degreesToRadians
from commands2 import SelectCommand, cmd
from wpimath.geometry import Rotation2d

from pathplannerlib.auto import AutoBuilder, PathPlannerPath, PathConstraints

from util import FalconLogger
from .DriveConstants import *
from subsystems import SwerveDrive


class DriveToPose(SelectCommand):
    __state = str

    constraints = PathConstraints(
        3.0, 4.0,
        degreesToRadians(540), degreesToRadians(720)
    )

    def __init__(self, sysDrive: SwerveDrive, state: str) -> None:
        self.__state = state
        super().__init__(
            {
                'Path': AutoBuilder.pathfindToPose(self.returnPose(), self.constraints, 0.0),
                # pathname or identifier: path
            },
            lambda: "Path"  # BotState.findNearestPath
            # cmd.none()

        )
        self.addRequirements(sysDrive)

    def returnPose(self) -> Pose2d | None:
        if self.__state == 'Right-Outer':
            return Pose2d(1.459, 0.79, Rotation2d.fromDegrees(45))
        elif self.__state == 'Reef State':
            return Pose2d(3.974, 2.83, Rotation2d.fromDegrees(45))
        return None

    # Is Finished
    def isFinished(self) -> bool:
        return super().isFinished()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
