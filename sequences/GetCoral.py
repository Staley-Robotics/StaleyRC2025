import math, typing

from commands2 import SequentialCommandGroup, ParallelCommandGroup

from subsystems import *
from commands import *


class GetCoral(SequentialCommandGroup):
    def __init__(self, coralManipulator: CoralManipulatorWheel, coralPivot: CoralManipulatorPivot, elevator: Elevator,
                 drive: SwerveDrive):
        super().__init__(
            ParallelCommandGroup(
                FollowPathSelect(drive, "Example Pickup"),
                ElevatorToPos(elevator, 0),
                SetPivotPosition(coralPivot, CoralManipulatorPivot.PivotPositions.SOURCE, 'Source'),
            ),
            SetCoralWheelSpeedIn(coralManipulator, CoralManipulatorWheel.WheelSpeeds.IN, 'Intake'),
        )

        self.setName("GetCoral")
        self.addRequirements(CoralManipulatorWheel, CoralManipulatorPivot, Elevator, SwerveDrive)
