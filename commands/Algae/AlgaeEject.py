from commands2 import Command, Subsystem
from subsystems.Algae import AlgaeManipulator, AlgaeManipulatorPositions, AlgaeIntakeState

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
            self.Algae.setSetpoint(AlgaeManipulatorPositions.PLACE)

    # Periodic
    def execute(self) -> None:
        if self.Algae.getMeasurement() >= self.Algae.getSetpoint():
            self.Algae.setIntake(AlgaeIntakeState.OUT)

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False# not self.Algae.hasAlgae()

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False