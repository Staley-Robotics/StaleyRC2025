import typing

from commands2 import Command
from subsystems import CoralManipulatorPivot, CoralPivotPositions

class SetPivotPosition(Command):
    # Variable Declaration
    pivot:CoralManipulatorPivot = None
    
    # Initialization
    def __init__( self,
                  pivot:CoralManipulatorPivot,
                  setpoint:CoralPivotPositions,
                  cmdID:str
                ) -> None:
        # Command Attributes
        self.pivot:CoralManipulatorPivot = pivot
        self.setpoint = setpoint

        self.setName( f'{self.__class__.__name__}:{cmdID}' )
        self.addRequirements( pivot )

    # On Start
    def initialize(self) -> None:
        self.pivot.setSetpoint( self.setpoint )

    # Periodic
    # def execute(self) -> None:
    #     print( f"Pivot({self.setpoint}) {self.pivot.getPosition()}")
    #     pass

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return self.pivot.atSetpoint()