import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber, ClimberConstants

from rev import SparkMax


class ClimberOpenControl(Command):

    # Initialization
    def __init__(self,
                 climberSubsystem: Climber,
                 getControl:typing.Callable[[], float]
                 ) -> None:
        # Command Attributes
        self.climber: Climber = climberSubsystem
        self.getControl = getControl

        self.setName("ClimberOpenControl")
        self.addRequirements(climberSubsystem)

    # On Start
    def initialize(self) -> None:
        self.climber.control_type = SparkMax.ControlType.kDutyCycle

    # Periodic
    def execute(self) -> None:
        self.climber.setSetpoint( self.getControl(), True )

    # On End
    def end(self, interrupted: bool) -> None:
        self.climber.control_type = SparkMax.ControlType.kDutyCycle
        self.climber.stop()

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
