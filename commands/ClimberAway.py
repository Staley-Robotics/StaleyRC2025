import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber, ClimberConstants


class ClimberClimb(Command):

    # Initialization
    def __init__(self,
                 climberSubsystem: Climber,
                 ) -> None:
        # Command Attributes
        self.Climber: Climber = climberSubsystem
        self.setName("ClimberClimb")
        self.addRequirements(climberSubsystem)

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.Climber.setPosition(90)

    # On End
    def end(self, interrupted: bool) -> None:
        self.Climber.safe_stop()

    # Is Finished
    def isFinished(self) -> bool:
        return self.Climber.atSetpoint()  # maybe used degrees or radians instead of rotations?

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
