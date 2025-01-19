import typing

from commands2 import Command, Subsystem
from subsystems.Climber import Climber

class ClimberClimb(Command):
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
        self.m_subsystem.setSpeed( self.m_subsystem.Speeds.FORWARD )

    # On End
    def end(self, interrupted:bool) -> None:
        self.m_subsystem.setSpeed( self.m_subsystem.Speeds.STOP)

    # Is Finished
    def isFinished(self) -> bool:
        # Def need to change this. Essentially I want to stop after climber rotates 90 degrees (or something idk)
        return self.m_subsystem.getPosition(unit="rotations") > .25

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False