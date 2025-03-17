import typing

from commands2 import Command, Subsystem
from subsystems.Algae import AlgaeManipulator, AlgaeManipulatorPositions, AlgaeIntakeState

class AlgaeGrab(Command):
    # Variable Declaration
    Algae:AlgaeManipulator = None

    # Initialization
    def __init__( self,
                  mySubsystem:AlgaeManipulator,
                ) -> None:
        # Command Attributes
        self.Algae:AlgaeManipulator = mySubsystem
        self.setName( "AlgaeGrab" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        self.Algae.setIntake(AlgaeIntakeState.IN)
        self.Algae.setSetpoint(AlgaeManipulatorPositions.GRAB)

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return self.Algae.hasAlgae()


    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False