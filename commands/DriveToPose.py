#import typing
from typing import List, Callable
import math

from pathplannerlib.path import GoalEndState
from wpilib import SmartDashboard, DriverStation

from wpimath import applyDeadband
from wpimath.geometry import Pose2d
from wpimath.units import degrees, degreesToRadians
from commands2 import Command, SequentialCommandGroup
from wpimath.geometry import Rotation2d

from pathplannerlib.auto import AutoBuilder, PathPlannerPath, PathConstraints

from util import FalconLogger, ReefScape
from util.ReefScape import *
from .DriveConstants import *
from subsystems import SwerveDrive


class DriveToPose(SequentialCommandGroup):
    constraints = PathConstraints(
        4.25, 4.25,
        degreesToRadians(720), degreesToRadians(720)
    )

    def __init__(self, sysDrive: SwerveDrive, poseGetter: Callable[[], Pose2d] = lambda: Pose2d(1,1, Rotation2d(0)) ) -> None:
        super().__init__()
        self.addRequirements(sysDrive)
        self.getPose = poseGetter
        self.setName( "DriveToPose" )

    def initialize(self) -> None:
        self.curPose = self.getPose()
        autoPath = AutoBuilder.pathfindToPoseFlipped( self.curPose, self.constraints, 0.0)
        self.addCommands( autoPath )
        super().initialize()

    def end(self, interrupted: bool) -> None:
        super().end(interrupted)
        self._commands: List[Command] = []

    def runsWhenDisabled(self) -> bool:
        return False
