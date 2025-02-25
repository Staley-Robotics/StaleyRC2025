# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

# Local Imports
from subsystems import Climber
from commands import ClimberNotClimb, ClimberClimb
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
        self.climber = Climber.Climber( 1 )

        # Autonomous Chooser
        self.__autoChooser.setDefaultOption( "1 - None", cmd.none() )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )

        # Default Commands
        # self.climber.setDefaultCommand(ClimberStay.ClimberStay(self.climber))

        # Driver Controller Button Binding
        driver1.a().whileTrue(ClimberClimb.ClimberClimb(self.climber)) # z on keyboard
        driver1.x().whileTrue(ClimberNotClimb.ClimberNotClimb(self.climber)) # c on keyboard

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
