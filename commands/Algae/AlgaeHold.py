from commands2 import Command, Subsystem
from subsystems.Algae import AlgaeManipulator, AlgaeManipulatorPositions, AlgaeIntakeState

class AlgaeHold(Command):
    # Variable Declaration
    m_subsystem:AlgaeManipulator = None

    # Initialization
    def __init__( self,
                  mySubsystem:Subsystem,
                ) -> None:
        # Command Attributes
        self.m_subsystem:AlgaeManipulator = mySubsystem
        self.setName( "AlgaeHold" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        self.m_subsystem.setIntake( AlgaeIntakeState.HOLD )
        self.m_subsystem.setSetpoint( AlgaeManipulatorPositions.HOLD )

    # Periodic
    def execute(self) -> None:
        if self.m_subsystem.hasAlgae():
            self.m_subsystem.setIntake( AlgaeIntakeState.HOLD )
        else:
            self.m_subsystem.setIntake( AlgaeIntakeState.OFF )

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return False

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False