# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd

from ntcore.util import ntproperty

# Local Imports
from subsystems import *
from commands import *
from sequences import *
from util import FalconXboxController, ReefScape

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
        control_mode: str = "Test"  # Can be "Comp", "Practice", "Test", "DriveOnly"

        ## Initialize State
        ReefScapeState = ReefScape.getInstance()

        ## Controllers
        self.driver1 = FalconXboxController(0, squaredInputs=True)
        self.driver2 = FalconXboxController(1, squaredInputs=True)

        ## Initialize Subsystems
        # self.sysDriveTrain = SwerveDrive()
        # self.sysVision = Vision( self.sysDriveTrain.getOdometry )
        self.sysElevator   = Elevator( 5, 6 )
        # self.sysCoralWheel = CoralManipulatorWheel( 8 )
        # self.sysCoralPivot = CoralManipulatorPivot( 7, 0.967803 )
        # self.sysAlgae      = AlgaeManipulator()
        # self.sysClimber    = Climber( 3, 4, 0.9024424 )
        # self.sysClimber    = ClimberSimple( 3, 4, 0.9136312 )

        # Vision Prep
        # AwaitVisionData( self.sysVision, self.sysDriveTrain ).schedule()


        ## Initialize Control Scheme
        ## Driver Controller Button Binding
        match control_mode:
            case "Comp":
                self.__bindCompetitionControls()
            case "Practice":
                self.__bindPracticeControls()
            case "Test":
                self.__bindTestControls()
            case "DriveOnly":
                self.__bindDriveOnly()

        # self.__autoChooser = AutoBuilder.buildAutoChooser("")
        # SmartDashboard.putData("AutoChooser", self.__autoChooser)


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
        ## Driver 1
        # DriveTrain
        self.sysDriveTrain.setDefaultCommand(
            DriveByStick( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide )
        )

        # Algae
        self.sysAlgae.setDefaultCommand( AlgaeHold( self.sysAlgae ) )
        self.driver1.a().whileTrue( AlgaeGrab( self.sysAlgae ) )
        self.driver1.b().whileTrue( AlgaeEject( self.sysAlgae ) ) # TODO: remake Algae eject as sequence to avoid early eject

        # Climber
        # self.sysClimber.setDefaultCommand( ClimberUp( self.sysClimber ) )
        # self.driver1.x().onTrue( ClimberOut( self.sysClimber ) )
        # self.driver1.y().toggleOnTrue( ClimberOpenControl( self.sysClimber, self.driver1.getTriggers ) )# NOTE: use self.driver1.getRightTriggerAxis ) ) to only allow open loop to contract climber
        self.sysClimber.setDefaultCommand( ClimberOpenControl( self.sysClimber, self.driver1.getTriggers ) )

        ## Driver 2 TODO: update for control board + controller
        # Coral Pivot
        # self.driver2.back().toggleOnTrue( ControlPivotPosition( self.sysCoralPivot, self.driver2.getLeftUpDown ) ) NOTE: if manual control wanted
        self.driver2.povDown().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L1, "L1" ) )
        self.driver2.povLeft().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L2, "L2" ) )
        self.driver2.povRight().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L3, "L3" ) )
        self.driver2.povUpLeft().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_up, "L4u" ) )
        self.driver2.povUpRight().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_down, "L4d" ) )
        # self.driver2.start().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.START, "Start" ) ) NOTE: what is this for?


        # Coral Wheel
        self.driver2.rightBumper().onTrue( CoralIO( self.sysCoralWheel ) )

        # Elevator
        # self.driver2.start().toggleOnTrue( ElevatorByStick( self.sysElevator, self.driver2.getRightUpDown ) ) NOTE: if manual control wanted
        self.driver2.a().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.BOTTOM ) )
        self.driver2.b().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.LOW_CORAL ) )
        self.driver2.x().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.MED_CORAL ) )
        self.driver2.y().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.HIGH_CORAL ) )

    def __bindTestControls(self):
        # # DriveTrain
        # self.sysDriveTrain.setDefaultCommand(
        #     DriveByStick( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide )
        # )
        #
        # # Algae
        # self.sysAlgae.setDefaultCommand( AlgaeHold( self.sysAlgae ) )
        # self.driver1.a().whileTrue( AlgaeGrab( self.sysAlgae ) )
        # self.driver1.b().whileTrue( AlgaeEject( self.sysAlgae ) )
        #
        # # Climber
        # self.sysClimber.setDefaultCommand( ClimberUp( self.sysClimber ) )
        # self.driver1.x().whileTrue( ClimberOut( self.sysClimber ) )
        # self.driver1.y().whileTrue( ClimberClimb( self.sysClimber ) )
        #
        # # Coral Pivot
        # #self.sysCoralPivot.setDefaultCommand()
        # self.driver1.povDown().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L1, "L1" ) )
        # self.driver1.povLeft().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L2, "L2" ) )
        # self.driver1.povRight().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L3, "L3" ) )
        # self.driver1.povUpLeft().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_up, "L4u" ) )
        # self.driver1.povUpRight().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_down, "L4d" ) )
        # self.driver1.start().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.START, "Start" ) )
        #
        # # Elevator
        # #self.sysElevator.setDefaultCommand()
        # self.driver2.a().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.BOTTOM ) )
        # self.driver2.b().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.LOW_CORAL ) )
        # self.driver2.x().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.MED_CORAL ) )
        # self.driver2.y().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.HIGH_CORAL ) )
        # # default commands
        # # defaults
        # #self.sysCoralPivot.setDefaultCommand(cmdCoralPivotControl)
        # # sysClimber.setDefaultCommand(cmdClimberStay)
        #
        # #self.sysDriveTrain.setDefaultCommand(cmdDriveByStick)
        #
        # ### Controls
        #
        # ## Coral
        # # self.driver1.x().onTrue(cmdCoralIn)
        # # self.driver1.y().onTrue(cmdCoralOut)
        # # # driver1.y().toggleOnTrue(cmdCoralIO)
        #
        #
        # ## CLimber
        # # driver2.y().toggleOnTrue(cmdClimberPosition)
        # # Only necessary climber commands (as of now): ClimberClimb (climber goes in), ClimberNotClimb (climber goes out), ClimberAway (climber goes straight up)
        #
        #
        # ## Elevator
        # # driver1.a().toggleOnTrue( cmdElevatorByStick )
        #
        # ## Algae

        self.sysElevator.setDefaultCommand(ElevatorByStick(self.sysElevator, self.driver1.getRightUpDown))
        self.driver1.a().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.TROUGH))
        self.driver1.b().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.LOW_CORAL))
        self.driver1.x().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.MED_CORAL))
        self.driver1.y().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.HIGH_CORAL))


    def __bindDriveOnly(self):
        self.sysDriveTrain.setDefaultCommand(
            DriveByStick( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide )
        )

        self.driver1.a().onTrue( cmd.runOnce(self.sysDriveTrain.resetOdometry) )