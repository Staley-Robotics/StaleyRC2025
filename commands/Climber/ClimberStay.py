import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber, ClimberConstants


class ClimberStay(Command):

    # Initialization
    def __init__(self,
                 climberSubsystem: Climber,
                 ) -> None:
        # Command Attributes
        self.Climber: Climber = climberSubsystem
        self.setName("ClimberStay")
        self.addRequirements(climberSubsystem)

    # On Start
    def initialize(self) -> None:
        self.Climber.setPosition(self.Climber.getPosition())

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
