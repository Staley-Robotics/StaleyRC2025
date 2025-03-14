from commands2 import ParallelCommandGroup

from subsystems import CoralManipulatorWheel, CoralManipulatorPivot
from commands import SetCoralWheelSpeed, SetPivotPosition

class CoralRelease(ParallelCommandGroup):
    def __init__(self, coralWheelSys:CoralManipulatorWheel):#, coralPivotSys:CoralManipulatorPivot):
        self.addCommands(
            SetCoralWheelSpeed( coralWheelSys, CoralManipulatorWheel.WheelSpeeds.OUT, 'Release' ),
            # SetPivotPosition( coralPivotSys, CoralManipulatorPivot.PivotPositions.SOURCE, 'Intake' )
        )