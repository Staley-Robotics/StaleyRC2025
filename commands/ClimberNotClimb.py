import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber, ClimberConstants

class ClimberNotClimb(Command):
    # Variable Declaration
    m_subsystem:Climber = None
    # m_getValue:typing.Callable[[],float] = lambda: 0.0
    
    # Initialization
    def __init__( self,
                  mySubsystem:Subsystem,
                #   myValue: typing.Callable[[], float] = lambda: 0.0
                ) -> None:
        # Command Attributes
        self.m_subsystem:Climber = mySubsystem
        # self.m_getValue = myValue
        self.setName( "SampleCommand" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.m_subsystem.setPosition(0)

    # On End
    def end(self, interrupted:bool) -> None:
        self.m_subsystem.stop()

    # Is Finished
    def isFinished(self) -> bool:
        # need to change this... don't want to just stop once I get to the right spot
        return self.m_subsystem.atSetpoint()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False