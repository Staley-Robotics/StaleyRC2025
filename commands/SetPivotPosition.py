import typing

from commands2 import Command, Subsystem
from subsystems import CoralManipulatorPivot

class SetPivotPosition(Command):
    # Variable Declaration
    pivot:CoralManipulatorPivot = None
    # m_getValue:typing.Callable[[],float] = lambda: 0.0
    
    # Initialization
    def __init__( self,
                  pivot:Subsystem,
                  myBaseValue: typing.Callable[[], float]
                ) -> None:
        # Command Attributes
        self.pivot:CoralManipulatorPivot = pivot
        self.m_getValue = myBaseValue

        self.setName( "SetPivotPosition" )
        self.addRequirements( pivot )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        #stupid scaling code 
        self.pivot.setSetpoint(self.m_getValue())# * (self.pivot.PivotConstants.max-self.pivot.PivotConstants.max)) +self.pivot.PivotConstants.min )

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False