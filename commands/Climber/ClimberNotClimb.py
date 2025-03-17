import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber, ClimberConstants


class ClimberNotClimb(Command):

    # Initialization
    def __init__(self,
                 climberSubsystem: Climber,
                 ) -> None:
        # Command Attributes
        self.Climber: Climber = climberSubsystem
        self.setName("ClimberNotClimb")
        self.addRequirements(climberSubsystem)

    # On Start
    def initialize(self) -> None:
        self.Climber.setPosition(-10.0)  

    # Periodic
    def execute(self) -> None:
        # self.Climber.setPosition(0)
        pass

    # On End
    def end(self, interrupted: bool) -> None:
        # self.Climber.safe_stop()
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False # self.Climber.atSetpoint()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
