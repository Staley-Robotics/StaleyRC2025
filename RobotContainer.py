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
        ## Controller Mapping Mode
        control_mode: str = "Test"  # Can be "Comp", "Practice", or "Test"

        ## Controllers
        self.driver1 = FalconXboxController(0, squaredInputs=True)
        self.driver2 = FalconXboxController(1, squaredInputs=True)

        ## Initialize Subsystems
        self.sysDriveTrain = SwerveDrive()
        self.sysElevator   = Elevator( 5, 6 )
        self.sysCoralWheel = CoralManipulatorWheel( 8 )
        self.sysCoralPivot = CoralManipulatorPivot( 7, 0.7794115 )
        self.sysAlgae      = AlgaeManipulator()
        self.sysClimber    = Climber( 3, 4, 0.9024424 )
        
        ## PathPlanner / Autonomous

        ## Initialize Control Scheme
        ## Driver Controller Button Binding
        match control_mode:
            case "Comp":
                self.__bindCompetitionControls()
            case "Practice":
                self.__bindPracticeControls()
            case "Test":
                self.__bindTestControls()


    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()

    def __bindCompetitionControls(self):
        pass

    def __bindPracticeControls(self):
        pass

    def __bindTestControls(self):
        # DriveTrain
        self.sysDriveTrain.setDefaultCommand(
            DriveByStick( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide )
        )

        # Algae
        self.sysAlgae.setDefaultCommand( AlgaeHold( self.sysAlgae ) )
        self.driver1.a().whileTrue( AlgaeGrab( self.sysAlgae ) )
        self.driver1.b().whileTrue( AlgaeHold( self.sysAlgae ) )

        # Climber
        self.sysClimber.setDefaultCommand( ClimberNotClimb( self.sysClimber ) )
        self.driver1.x().whileTrue( ClimberAway( self.sysClimber ) ) 
        self.driver1.y().whileTrue( ClimberClimb( self.sysClimber ) )
        
        # Coral Pivot
        #self.sysCoralPivot.setDefaultCommand()
        self.driver1.povDown().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L1, "L1" ) )
        self.driver1.povLeft().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L2, "L2" ) )
        self.driver1.povRight().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L3, "L3" ) )
        self.driver1.povUpLeft().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_up, "L4u" ) )
        self.driver1.povUpRight().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_down, "L4d" ) )
        self.driver1.start().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.START, "Start" ) )
        
        # Elevator
        #self.sysElevator.setDefaultCommand()
        self.driver2.a().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.BOTTOM ) )
        self.driver2.b().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.LOW_CORAL ) )
        self.driver2.x().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.MED_CORAL ) )
        self.driver2.y().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.HIGH_CORAL ) )
        # default commands
        # defaults
        #self.sysCoralPivot.setDefaultCommand(cmdCoralPivotControl)
        # sysClimber.setDefaultCommand(cmdClimberStay)
        
        #self.sysDriveTrain.setDefaultCommand(cmdDriveByStick)

        ### Controls

        ## Coral
        # self.driver1.x().onTrue(cmdCoralIn)
        # self.driver1.y().onTrue(cmdCoralOut)
        # # driver1.y().toggleOnTrue(cmdCoralIO)


        ## CLimber
        # driver2.y().toggleOnTrue(cmdClimberPosition)
        # Only necessary climber commands (as of now): ClimberClimb (climber goes in), ClimberNotClimb (climber goes out), ClimberAway (climber goes straight up)
        

        ## Elevator
        # driver1.a().toggleOnTrue( cmdElevatorByStick )

        ## Algae

