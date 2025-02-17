# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

# Local Imports
from subsystems import *
from commands import *
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
        sysCoralManipulatorPivot = CoralManipulatorPivot( 'CoralManipulatorPivot', 12, 20 )
        sysCoralManipulatorWheel = CoralManipulatorWheel( 'CoralManipulatorWheel', 3 )

        # Commands
        cmdSetPivotPosition = SetPivotPosition( sysCoralManipulatorPivot, lambda: driver1.getLeftUpDown())
        cmdRunCoralWheel = CoralWheelOpenControl( sysCoralManipulatorWheel, driver1.getRightUpDown )
        

        # Autonomous Chooser
        self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
        sysCoralManipulatorPivot.setDefaultCommand( cmdSetPivotPosition )
        sysCoralManipulatorWheel.setDefaultCommand( cmdRunCoralWheel )

        # Driver Controller Button Binding


    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
