from commands2 import Command, Subsystem
from subsystems.Algae import AlgaeManipulator, AlgaeManipulatorPositions, IntakeState

class AlgaeEject(Command):
    # Variable Declaration
    Algae:AlgaeManipulator = None

    # Initialization
    def __init__( self,
                  mySubsystem:Subsystem,
                ) -> None:
        # Command Attributes
        self.Algae:AlgaeManipulator = mySubsystem
        self.setName( "AlgaeEject" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        if not self.Algae.hasAlgae():
            self.cancel()
        else:
            self.Algae.setIntake(IntakeState.OUT)
            self.Algae.setSetpoint(AlgaeManipulatorPositions.PLACE)

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return not self.Algae.hasAlgae()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False