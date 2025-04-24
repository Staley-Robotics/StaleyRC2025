import typing

from commands2 import Command, Subsystem
from subsystems.ClimberOld import ClimberOld, ClimberConstants


class ClimberStay(Command):

    # Initialization
    def __init__(self,
                 climberSubsystem: ClimberOld,
                 ) -> None:
        # Command Attributes
        self.Climber: ClimberOld = climberSubsystem
        self.setName("ClimberStay")
        self.addRequirements(climberSubsystem)

    # On Start
    def initialize(self) -> None:
        self.Climber.setSetpoint(self.Climber.getPosition())

    # Periodic
    def execute(self) -> None:
        ...

    # On End
    def end(self, interrupted: bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
