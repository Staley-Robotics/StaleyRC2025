import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber, ClimberConstants

class ClimberClimb(Command):
    
    # Initialization
    def __init__( self,
                  mySubsystem:Climber,
                ) -> None:
        
        # Command Attributes
        self.m_subsystem:Climber = mySubsystem
        self.setName( "ClimberClimb" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.m_subsystem.setSpeed(ClimberConstants.Speeds.FORWARD)

    # On End
    def end(self, interrupted:bool) -> None:
        self.m_subsystem.stop()

    # Is Finished
    def isFinished(self) -> bool:
        return self.m_subsystem.reachedClimbedLimit()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False