import math, typing

from commands2 import SequentialCommandGroup, ParallelCommandGroup

from subsystems import *
from commands import *
from util import ReefScape


class GetCoral(SequentialCommandGroup):
    def __init__(self, drive: SwerveDrive):
        super().__init__(
            DriveToPose(drive, ReefScape.getCoralPose),
        )

        self.setName("GetCoral")
        self.addRequirements(drive)
