import typing
import math

from pathplannerlib.path import GoalEndState
from wpilib import SmartDashboard, DriverStation, RobotBase

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

    def __init__(self, sysDrive: SwerveDrive, poseGetter: Callable[[], Pose2d] = lambda: Pose2d(1,1, Rotation2d(0))) -> None:
        super().__init__()
        self.getPose = poseGetter
        self.sysDrive = sysDrive
        self.addRequirements(sysDrive)
        self.setName("DriveToPose")

    def initialize(self) -> None:
        target_pose = self.getPose()
        current_pose = self.sysDrive.getPose()

        # Safeguard.   This command breaks in simulation if robot is out of bounds.
        if current_pose.X() < 0.0 or current_pose.Y() < 0.0 or current_pose.X() > 16.0 or current_pose.Y() > 8.0:
            return

        __pathCommand = AutoBuilder.pathfindToPoseFlipped( target_pose, self.constraints, 0.0) #.hasRequirement(self.sysDrive)
        #self._commands: list[Command] = []
        self.addCommands( __pathCommand )
        super().initialize()

    def end(self, interrupted: bool) -> None:
        super().end(interrupted)
        self._commands: list[Command] = []

    # def isFinished(self):
    #     return super().isFinished()

