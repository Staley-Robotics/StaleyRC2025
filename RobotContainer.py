# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

# Local Imports
from subsystems import SampleSubsystem, Elevator
from commands import SampleCommand, ElevatorToPos
from util import FalconXboxController

class RobotContainer:
    """
    RobotContainer is the Initial Container for an FRC Robot
    """
    # Variable Declaration
    __autoChooser:SendableChooser = SendableChooser()

    # Initialization
    def __init__(self):
        """
        Initializes RobotContainer
        """
        # Driver Controller
        driver1 = FalconXboxController( 0 )

        # Declare Subsystems
        sysSample = SampleSubsystem( 0 )
        elevator = Elevator()

        # Commands
        cmdSampleLeft = SampleCommand(sysSample, driver1.getLeftX )
        cmdSampleRight = SampleCommand(sysSample, driver1.getRightX )
        cmdElevatorTo10 = ElevatorToPos(elevator, 10)
        cmdElevatorTo0 = ElevatorToPos(elevator, 0)

        # Autonomous Chooser
        self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
        sysSample.setDefaultCommand( cmdSampleLeft )

        # Driver Controller Button Binding
        driver1.a().whileTrue( cmdSampleRight )
        driver1.b().onTrue(cmdElevatorTo0)
        driver1.x().onTrue(cmdElevatorTo10)


    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
