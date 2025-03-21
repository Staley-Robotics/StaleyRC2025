import typing

from commands2 import Command, Subsystem
from subsystems import CoralManipulatorPivot
from ntcore.util import ntproperty

class ControlPivotPosition(Command):
    # Variable Declaration
    pivot:CoralManipulatorPivot = None
    # m_getValue:typing.Callable[[],float] = lambda: 0.0
    controlMult = ntproperty('/Setting/CoralManipulatoPivot/ControlSpeed', 1)
    
    # Initialization
    def __init__( self,
                  pivot:Subsystem,
                  getValueChange: typing.Callable[[], float],
                ) -> None:
        # Command Attributes
        self.pivot:CoralManipulatorPivot = pivot
        self.m_getValue = getValueChange
        # self.setpoint = self.pivot.getPosition()

        self.setName( "ControlPivotPosition" )
        self.addRequirements( pivot )

    # On Start
    def initialize(self) -> None:
        # self.setpoint = self.pivot.getPosition()
        ...

    # Periodic
    def execute(self) -> None:
        # self.setpoint = min(max(self.setpoint + self.m_getValue(), CoralManipulatorPivot.PivotPositions.MIN ), CoralManipulatorPivot.PivotPositions.MAX)
        new_val = self.pivot.getSetpoint() + self.m_getValue() * self.controlMult

        if new_val > 180:
            new_val -= 360
        elif new_val < -180:
            new_val += 360

        self.pivot.setSetpoint(new_val)

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False