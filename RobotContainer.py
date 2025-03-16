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
        # sysCoralPivot = CoralManipulatorPivot( 7, 0.7794115 )
        # sysClimber = Climber(3, 4, 0.9024424)
        driver1 = FalconXboxController(0, squaredInputs=True)
        driver2 = FalconXboxController(1, squaredInputs=True)
        # sysElevator = Elevator( 5, 6, driver1.getRightUpDown )
        # sysAlgae = AlgaeManipulator()
        sysDriveTrain = SwerveDrive()

        ## Initialize Commands
        # create controller objects

        ### Commands
        ## Coral
        # cmdCoralIO = CoralIO(sysCoralWheel)
        # cmdSetPivotl1 = SetPivotPosition( sysCoralPivot, sysCoralPivot.PivotPositions.L1, "SetPivotSource" )
        # cmdSetPivotL2 = SetPivotPosition( sysCoralPivot, sysCoralPivot.PivotPositions.L2, "SetPivotHold" )
        # cmdSetPivotL3 = SetPivotPosition( sysCoralPivot, sysCoralPivot.PivotPositions.L3, "SetPivotL3" )

        # cmdCoralIn = CoralWheelIn( sysCoralWheel )
        # cmdCoralOut = CoralWheelOut( sysCoralWheel )
        # cmdCoralPivotControl = ControlPivotPosition( sysCoralPivot, driver1.getRightUpDown )

        ## Climber
        # cmdClimberClimb = ClimberClimb( sysClimber )
        # cmdClimberNotClimb = ClimberNotClimb( sysClimber )
        # cmdClimberPosition = ClimberPosition( sysClimber, driver1.getLeftUpDown )
        # cmdClimberAway = ClimberNotClimb( sysClimber )
        # cmdClimberStay = ClimberStay( sysClimber )
        
        ## Elevator
        # cmdElevatorByStick = ElevatorByStick( sysElevator, driver1.getRightUpDown )

        ## DriveTrain
        cmdDriveByStick = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )

        ## Algae
        # cmdAlgaeGrab = AlgaeGrab( sysAlgae )
        # cmdAlgaeEject = AlgaeEject( sysAlgae )
        # cmdAlgaeHold = AlgaeHold( sysAlgae )

        # default commands
        # defaults
        # sysCoralPivot.setDefaultCommand(cmdCoralPivotControl)
        # sysClimber.setDefaultCommand(cmdClimberStay)
        # sysAlgae.setDefaultCommand(AlgaeHold( sysAlgae ))
        sysDriveTrain.setDefaultCommand(cmdDriveByStick)


        ### Controls
        ## Coral
        # driver1.x().onTrue(cmdCoralIn)
        # driver1.y().onTrue(cmdCoralOut)
        # # driver1.y().toggleOnTrue(cmdCoralIO)

        # driver1.pov(0).onTrue(cmdSetPivotl1)
        # driver1.pov(45).onTrue(cmdSetPivotL2)
        # driver1.pov(90).onTrue(cmdSetPivotL3)
        # driver1.a().toggleOnTrue(cmdCoralPivotControl)

        ## CLimber
        # driver1.y().toggleOnTrue(cmdClimberPosition)
        # driver1.a().onTrue(cmdClimberClimb) # z on keyboard
        # driver1.x().onTrue(cmdClimberNotClimb) # c on keyboard

        ## Elevator
        # driver1.a().toggleOnTrue( cmdElevatorByStick )

        ## Algae
        # driver1.a().whileTrue(cmdAlgaeGrab)
        # driver1.b().whileTrue(cmdAlgaeEject)


    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
