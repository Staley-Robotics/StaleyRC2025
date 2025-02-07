from commands2 import Command, Subsystem
from subsystems.AlgaeSubsystem import AlgaeManipulator, AlgaeManipulatorPositions, IntakeState

class AlgaeEjectCommand(Command):
    # Variable Declaration
    m_subsystem:AlgaeManipulator = None

    # Initialization
    def __init__( self,
                  mySubsystem:Subsystem,
                ) -> None:
        # Command Attributes
        self.m_subsystem:AlgaeManipulator = mySubsystem
        self.setName( "AlgaeEject" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        self.m_subsystem.setIntake( IntakeState.OUT )
        self.m_subsystem.setSetpoint( AlgaeManipulatorPositions.PLACE )

    # Periodic
    def execute(self) -> None:
        pass

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        if self.m_subsystem.atSetpoint():
            return True
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False