import typing
import math

from wpilib import SmartDashboard

from wpimath import applyDeadband
from wpimath.units import degrees
from commands2 import SelectCommand, cmd

from pathplannerlib.auto import AutoBuilder, PathPlannerPath

from util import FalconLogger
from .DriveConstants import *
from subsystems import SwerveDrive


class FollowPathSelect(SelectCommand):
    __path = str

    def __init__(self, sysDrive: SwerveDrive, path: str) -> None:
        self.__path = path
        super().__init__(
            {
                'A to A1': AutoBuilder.followPath(PathPlannerPath.fromPathFile('A to A1')),
                'Example Pickup': AutoBuilder.followPath(PathPlannerPath.fromPathFile('Example Pickup')),
                # pathname or identifier: path
            },
            self.choosePath  # BotState.findNearestPath
            # cmd.none()
        )
        '''
        format:
        {key:command},
        () -> key,
        defaultCommand (leave empty for print err command)
        '''
        self.addRequirements(sysDrive)

    def choosePath(self) -> str:
        return self.__path

    # Is Finished
    def isFinished(self) -> bool:
        return super().isFinished()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
