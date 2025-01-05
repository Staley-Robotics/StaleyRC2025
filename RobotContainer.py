# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

# Local Imports
from subsystems import *
from commands import *
from util import FalconXboxController

try:
    from pathplannerlib.auto import AutoBuilder, NamedCommands
except Exception as e:
    print("ERROR: PlanPlanner Includes")
    print( e )

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
        sysDriveTrain = SwerveDrive()

        # Commands
        cmdDriveByStick = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )

        # Autonomous Chooser
        # self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        # SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
        sysDriveTrain.setDefaultCommand( cmdDriveByStick )

        # PathPlanner Setup
        try:
            # PathPlanner Register Named Commands
            NamedCommands.registerCommand('Pickup', cmd.waitSeconds(0.25) )
            NamedCommands.registerCommand('LaunchSpeaker', cmd.waitSeconds(0.25) )

            # Autonomous Chooser
            self.__autoChooser = AutoBuilder.buildAutoChooser( "None" )
            SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )
        except Exception as e:
            print( "ERROR: PathPlanner Named Commands and Chooser")
            print( e )

        # Driver Controller Button Binding
        # driver1.a().whileTrue( cmdSampleRight )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
