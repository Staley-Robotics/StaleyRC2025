# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

# Local Imports
from subsystems import *
from commands import *
from util import FalconXboxController

from pathplannerlib.auto import AutoBuilder, NamedCommands

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
        sysVisionCamera = VisionCameraDetector(1)
        sysDriveTrain = SwerveDrive()

        # Commands
        cmdDriveByStick = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )
        cmdDriveByNote = DriveByNote( sysDriveTrain, sysVisionCamera, driver1.getLeftUpDown, driver1.getLeftSideToSide )

        # Autonomous Chooser
        self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
        sysDriveTrain.setDefaultCommand( cmdDriveByStick )

        # PathPlanner Setup
        # PathPlanner Register Named Commands
        NamedCommands.registerCommand('Pickup', cmd.waitSeconds(0.25) )
        NamedCommands.registerCommand('LaunchSpeaker', cmd.waitSeconds(0.25) )

        # Autonomous Chooser
        self.__autoChooser = AutoBuilder.buildAutoChooser( "None" )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Driver Controller Button Binding
        driver1.a().toggleOnTrue( cmdDriveByNote )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
