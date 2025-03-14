import math, typing

from commands2 import SequentialCommandGroup, ParallelCommandGroup

from subsystems import *
from commands import *
from util import ReefScape


class GetCoral(SequentialCommandGroup):
    def __init__(self, coralManipulator: CoralManipulatorWheel, coralPivot: CoralManipulatorPivot, elevator: Elevator,
                 drive: SwerveDrive):
        super().__init__(
            ParallelCommandGroup(
                # FollowPathSelect(drive, "Example Pickup"),
                DriveToPose(drive),
                ElevatorToPos(elevator, 0),
                SetPivotPosition(coralPivot, CoralManipulatorPivot.PivotPositions.SOURCE, 'Source'),
            ),
            CoralWheelIn(coralManipulator),
        )

        self.setName("GetCoral")
        self.addRequirements(coralManipulator, coralPivot, elevator, drive)
