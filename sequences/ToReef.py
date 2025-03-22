import math, typing

from commands2 import SequentialCommandGroup, ParallelCommandGroup

from subsystems import *
from commands import *
from util import ReefScape


class ToReef(SequentialCommandGroup):
    def __init__(self, coralManipulator: CoralManipulatorWheel, coralPivot: CoralManipulatorPivot, elevator: Elevator,
                 drive: SwerveDrive):
        super().__init__(
            DriveToPose(drive, ReefScape.getReefPose)
        )

        self.setName("ToReef")
        self.addRequirements(coralManipulator, coralPivot, elevator, drive)
