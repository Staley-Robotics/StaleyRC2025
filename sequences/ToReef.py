import math, typing

from commands2 import SequentialCommandGroup, ParallelCommandGroup

from subsystems import *
from commands import *


class ToReef(SequentialCommandGroup):
    def __init__(self, coralManipulator: CoralManipulatorWheel, coralPivot: CoralManipulatorPivot, elevator: Elevator,
                 drive: SwerveDrive):
        super().__init__(
            ParallelCommandGroup(
                FollowPathSelect(drive, 'A to A1'),
                ElevatorToPos(elevator, 20),
                SetPivotPosition(coralPivot, CoralManipulatorPivot.PivotPositions.L2, 'L2'),
            ),
            SetCoralWheelSpeedOut(coralManipulator, CoralManipulatorWheel.WheelSpeeds.OUT, 'OUT'),
        )

        self.setName("ToReef")
        self.addRequirements([CoralManipulatorWheel, CoralManipulatorPivot, Elevator, SwerveDrive])
