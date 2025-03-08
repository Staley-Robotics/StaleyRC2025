import typing
import math

from pathplannerlib.path import GoalEndState
from wpilib import SmartDashboard, DriverStation

from wpimath import applyDeadband
from wpimath.geometry import Pose2d
from wpimath.units import degrees, degreesToRadians
from commands2 import Command
from wpimath.geometry import Rotation2d

from pathplannerlib.auto import AutoBuilder, PathPlannerPath, PathConstraints

from util import FalconLogger, ReefScape, SourceSide, SourceSelect
from .DriveConstants import *
from subsystems import SwerveDrive


class DriveToPose(Command):
    constraints = PathConstraints(
        3.0, 4.0,
        degreesToRadians(540), degreesToRadians(720)
    )

    def __init__(self, sysDrive: SwerveDrive) -> None:
        super().__init__()
        self.__drive = sysDrive
        self.__pathCommand = None
        self.addRequirements(sysDrive)

    def initialize(self) -> None:
        # Create path with CURRENT state when command starts
        target_pose = self.returnPose()

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.__pathCommand = AutoBuilder.pathfindToPoseFlipped(target_pose, self.constraints, 0.0)
        else:
            self.__pathCommand = AutoBuilder.pathfindToPose(target_pose, self.constraints, 0.0)
        self.__pathCommand.initialize()

    def execute(self) -> None:
        if self.__pathCommand is not None:
            self.__pathCommand.execute()

    def end(self, interrupted: bool) -> None:
        if self.__pathCommand is not None:
            self.__pathCommand.end(interrupted)
            self.__pathCommand = None

    def isFinished(self) -> bool:
        if self.__pathCommand is not None:
            return self.__pathCommand.isFinished()
        return True

    def returnPose(self) -> Pose2d:
        # Access ReefScape through class methods
        if ReefScape.getInstance().getHasCoral():
            return Pose2d(3.974, 2.83, Rotation2d.fromDegrees(45))
        else:
            source_side = ReefScape.getInstance().getSourceSide()
            source_select = ReefScape.getInstance().getSourceSelect()

            if source_side == SourceSide.RIGHT:
                if source_select == SourceSelect.OUTER:
                    return Pose2d(1.459, 0.79, Rotation2d.fromDegrees(46.5))
                elif source_select == SourceSelect.MIDDLE:
                    return Pose2d(1.202, 1.029, Rotation2d.fromDegrees(46.5))
                elif source_select == SourceSelect.INNER:
                    return Pose2d(0.734, 1.340, Rotation2d.fromDegrees(46.5))
            elif source_side == SourceSide.LEFT:
                if source_select == SourceSelect.OUTER:
                    return Pose2d(1.7, 7.413, Rotation2d.fromDegrees(-27))
                elif source_select == SourceSelect.MIDDLE:
                    return Pose2d(1.2, 7.051, Rotation2d.fromDegrees(-27))
                elif source_select == SourceSelect.INNER:
                    return Pose2d(0.660, 6.681, Rotation2d.fromDegrees(-27))

        return Pose2d(3.974, 2.83, Rotation2d.fromDegrees(45))

    def runsWhenDisabled(self) -> bool:
        return False
