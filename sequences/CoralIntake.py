from commands2 import ParallelCommandGroup

from subsystems import CoralManipulatorWheel, CoralManipulatorPivot
from commands import SetCoralWheelSpeed, SetPivotPosition

class CoralIntake(ParallelCommandGroup):
    def __init__(self, coralWheelSys:CoralManipulatorWheel, coralPivotSys:CoralManipulatorPivot):
        self.addCommands(
            SetCoralWheelSpeed( coralWheelSys, CoralManipulatorWheel.WheelSpeeds.IN, 'Intake' ),
            SetPivotPosition( coralPivotSys, CoralManipulatorPivot.PivotPositions.SOURCE, 'Intake' )
        )