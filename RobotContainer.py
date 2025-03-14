# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

# Local Imports
#from subsystems import SampleSubsystem
from subsystems import *
#from commands import SampleCommand
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
        self.sysSwerve = SwerveTest()

        # Driver Controller Button Binding
        driver1.leftBumper().onTrue( cmd.runOnce( self.toggleDefaults ) )

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()

    def toggleDefaults(self):
        if self.sysSwerve.getDefaultCommand() != None:
            self.sysSwerve.removeDefaultCommand()
            self.sysSwerve.getCurrentCommand().cancel()
            # add next
        else:
            self.sysSwerve.setDefaultCommand( SwerveDefault( self.sysSwerve ) )