import math, typing

from commands2 import SequentialCommandGroup, ParallelCommandGroup

from subsystems import *
from commands import *


class ToReef(SequentialCommandGroup):
    def __init__(self, coralManipulator: CoralManipulatorWheel, coralPivot: CoralManipulatorPivot, elevator: Elevator,
                 drive: SwerveDrive):
        super().__init__(
            ParallelCommandGroup(
                DriveToPose(drive),
                ElevatorToPos(elevator, 20),
                SetPivotPosition(coralPivot, CoralPivotPositions.L2, 'L2'),
            ),
            CoralWheelOut(coralManipulator, CoralManipulatorWheel.WheelSpeeds.OUT, 'OUT'),
        )

        self.setName("ToReef")
        self.addRequirements(coralManipulator, coralPivot, elevator, drive)
