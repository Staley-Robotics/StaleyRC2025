# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

from ntcore.util import ntproperty
from commands.AlgaeEject import AlgaeEjectCommand

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
        # Driver Controller
        driver1 = FalconXboxController( 0, squaredInputs=ntproperty("/Settings/Driver1/SquaredInputs", True) )

        # Declare Subsystems

        sysCoralManipulatorPivot = CoralManipulatorPivot( 'CoralManipulatorPivot', 12, 20 )
        sysCoralManipulatorWheel = CoralManipulatorWheel( 'CoralManipulatorWheel', 3 )

        sysAlgae = AlgaeManipulator()

        # Commands
        # cmdSampleLeft = SampleCommand(sysSample, driver1.getLeftX )
        cmdSetPivotPosition = SetPivotPosition( sysCoralManipulatorPivot, lambda: driver1.getLeftUpDown())
        cmdRunCoralWheel = CoralWheelOpenControl( sysCoralManipulatorWheel, driver1.getRightUpDown )
        
        cmdAlgaeGrab = AlgaeGrabCommand( sysAlgae )
        cmdAlgaeDefault = AlgaeHoldCommand( sysAlgae )
        cmdAlgaeEject = AlgaeEjectCommand( sysAlgae )
        
        sysDriveTrain = SwerveDrive()
        sysVision = Vision( sysDriveTrain.getOdometry )

        # Commands
        cmdDriveByStick = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )
        cmdAwaitVisionData = AwaitVisionData( sysVision.has_recieved_first_botpose_data, sysDriveTrain.resetOdometry, sysVision.get_last_pose )

        # Autonomous Chooser
        # self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        # SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
        sysCoralManipulatorPivot.setDefaultCommand( cmdSetPivotPosition )
        sysCoralManipulatorWheel.setDefaultCommand( cmdRunCoralWheel )
        sysAlgae.setDefaultCommand( cmdAlgaeDefault )
        sysDriveTrain.setDefaultCommand( cmdDriveByStick )
        cmdAwaitVisionData.schedule()

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
        driver1.a().whileTrue( cmdAlgaeGrab )
        driver1.b().whileTrue( cmdAlgaeEject )
        driver1.back().onTrue( cmd.runOnce( sysDriveTrain.resetOdometry() ) )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
