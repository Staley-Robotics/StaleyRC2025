import typing

from commands2 import Command, Subsystem
from subsystems import Climber
from ntcore.util import ntproperty

class ClimberPosition(Command):
    # Variable Declaration
    climber:Climber = None
    # m_getValue:typing.Callable[[],float] = lambda: 0.0
    controlMult = ntproperty('/Setting/Climber/ControlSpeed', 1)
    
    # Initialization
    def __init__( self,
                  sys:Subsystem,
                  getValueChange: typing.Callable[[], float],
                ) -> None:
        # Command Attributes
        self.climber:Climber = sys
        self.m_getValue = getValueChange
        # self.setpoint = self.pivot.getPosition()

        self.setName( "ClimberPosition" )
        self.addRequirements( sys )

    # On Start
    def initialize(self) -> None:
        # self.setpoint = self.pivot.getPosition()
        ...

    # Periodic
    def execute(self) -> None:
        # self.setpoint = min(max(self.setpoint + self.m_getValue(), CoralManipulatorPivot.PivotPositions.MIN ), CoralManipulatorPivot.PivotPositions.MAX)
        new_val = self.climber.getPosition() + self.m_getValue() * self.controlMult

        if new_val > 270:
            new_val -= 360
        elif new_val < -90:
            new_val += 360

        self.climber.setPosition(new_val)

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False