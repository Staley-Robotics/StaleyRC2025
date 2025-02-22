import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber, ClimberConstants

class ClimberNotClimb(Command):
    
    # Initialization
    def __init__( self,
                  mySubsystem:Subsystem,
                ) -> None:
        # Command Attributes
        self.m_subsystem:Climber = mySubsystem
        self.setName( "ClimberNotClimb" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.m_subsystem.setPosition(0)

    # On End
    def end(self, interrupted:bool) -> None:
        self.m_subsystem.safe_stop()

    # Is Finished
    def isFinished(self) -> bool:
        return self.m_subsystem.atSetpoint()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False