from commands2 import ParallelCommandGroup

from subsystems import CoralManipulatorWheel, CoralManipulatorPivot
from commands import SetCoralWheelSpeed, SetPivotPosition

class CoralHold(ParallelCommandGroup):
    def __init__(self, coralWheelSys:CoralManipulatorWheel, coralPivotSys:CoralManipulatorPivot):
        self.addCommands(
            SetCoralWheelSpeed( coralWheelSys, CoralManipulatorWheel.WheelSpeeds.STOP, 'Hold' ),
            SetPivotPosition( coralPivotSys, CoralManipulatorPivot.PivotPositions.HOLD, 'Hold' )
        )