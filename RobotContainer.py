# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

from ntcore.util import ntproperty

# Local Imports
from subsystems import *
from commands import *
from sequences import *
from util import FalconXboxController, ReefScape, SourceSelect, SourceSide

from pathplannerlib.auto import AutoBuilder, NamedCommands, PathPlannerPath

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

        '''
        Testing Logging:
        - order: coral wheel, coral pivot, algae, climber, elevator
        '''
        ## Initialize Subsystems
        # sysCoralWheel = CoralManipulatorWheel( 8 )
        sysCoralPivot = CoralManipulatorPivot( 7, 0.7794115 )

        ## Initialize Commands
        # create controller objects
        driver1 = FalconXboxController(0, squaredInputs=True)
        driver2 = FalconXboxController(1, squaredInputs=True)

        # commands
        # cmdCoralIO = CoralIO(sysCoralWheel)
        cmdSetPivotl1 = SetPivotPosition( sysCoralPivot, sysCoralPivot.PivotPositions.L1, "SetPivotSource" )
        cmdSetPivotL2 = SetPivotPosition( sysCoralPivot, sysCoralPivot.PivotPositions.L2, "SetPivotHold" )
        cmdSetPivotL3 = SetPivotPosition( sysCoralPivot, sysCoralPivot.PivotPositions.L3, "SetPivotL3" )

        # cmdCoralIn = CoralWheelIn( sysCoralWheel )
        # cmdCoralOut = CoralWheelOut( sysCoralWheel )

        # defaults
        # sysCoralPivot.setDefaultCommand(cmdCoralPivotControl)


        ## Controls
        

        # assign controls
        # driver1.a().onTrue(cmdCoralIn)
        # driver1.y().onTrue(cmdCoralOut)
        driver1.pov(0).onTrue(cmdSetPivotl1)
        driver1.pov(45).onTrue(cmdSetPivotL2)
        driver1.pov(90).onTrue(cmdSetPivotL3)


    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
