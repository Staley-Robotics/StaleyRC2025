# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

from ntcore.util import ntproperty

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

    # Initializationn
    def __init__(self):
        """
        Initializes RobotContainer
        """
        ## Driver Controller
        driver1 = FalconXboxController( 0, squaredInputs=ntproperty("/Settings/Driver1/SquaredInputs", True) )

        ## Declare Subsystems
        sysDriveTrain = SwerveDrive()
        sysVision = Vision( sysDriveTrain.getOdometry )

        ## Commands
        cmdDriveByStick = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )
        cmdDriveByStickRotate = DriveByStickRotate( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getHID().getPOV )
        cmdAwaitVisionData = AwaitVisionData( lambda: sysVision.has_recieved_data, sysDriveTrain.resetOdometry, sysVision.get_last_pose )

        ## Default Commands
        sysDriveTrain.setDefaultCommand( cmdDriveByStickRotate )
        cmdAwaitVisionData.schedule()

        ## Driver Controller Button Binding
        driver1.back().onTrue( cmd.runOnce( sysDriveTrain.resetOdometry() ) )

        ## PathPlanner Setup
        # PathPlanner Register Named Commands
        NamedCommands.registerCommand('Pickup', cmd.waitSeconds(0.25) )
        NamedCommands.registerCommand('LaunchSpeaker', cmd.waitSeconds(0.25) )

        # Autonomous Chooser
        self.__autoChooser = AutoBuilder.buildAutoChooser( "None" )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
