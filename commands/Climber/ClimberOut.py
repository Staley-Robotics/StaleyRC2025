import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber, ClimberConstants, ClimberPositions

from rev import SparkMax


class ClimberOut(Command):

    # Initialization
    def __init__(self,
                 climberSubsystem: Climber,
                 ) -> None:
        # Command Attributes
        self.Climber: Climber = climberSubsystem
        self.setName("ClimberOut")
        self.addRequirements(climberSubsystem)

        self.setpoint = ClimberPositions.Prepare

    # On Start
    def initialize(self) -> None:
        self.Climber.control_type = SparkMax.ControlType.kPosition
        self.Climber.setSetpoint( ClimberPositions.Prepare )  
        ...

    # Periodic
    def execute(self) -> None:
        # self.Climber.__controller
        ...

    # On End
    def end(self, interrupted: bool) -> None:
        self.Climber.control_type = SparkMax.ControlType.kDutyCycle
        self.Climber.setSetpoint( 0.0, True )
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return self.Climber.atSetpoint()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False
