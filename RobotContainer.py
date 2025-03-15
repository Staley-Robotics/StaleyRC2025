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

        ## Declare Subsystems
        # sysDriveTrain = SwerveDrive()
        # sysVision = Vision( sysDriveTrain.getOdometry )

        sysCoralManipulatorPivot = CoralManipulatorPivot( 7 )
        sysCoralManipulatorWheel = CoralManipulatorWheel( 8 )

        # sysAlgae = AlgaeManipulator()
        # sysElevator = Elevator()
        # sysClimber = Climber( 15 )

        ## Configure State
        # ReefScapeState = ReefScape.getInstance()
        # ReefScapeState.setHasCoral( sysCoralManipulatorWheel.hasCoral )
        # ReefScapeState.setGetRobotPose( sysDriveTrain.getPose )
        # ReefScapeState.setHasAlgae( sysAlgae.hasAlgae )

        # Put Subsystems on NetworkTables
        # SmartDashboard.putData( 'DriveTrain', sysDriveTrain )
        # SmartDashboard.putData( 'Vision', sysVision )
        SmartDashboard.putData( 'CoralPivot', sysCoralManipulatorPivot )
        SmartDashboard.putData( 'CoralWheel', sysCoralManipulatorWheel )
        # SmartDashboard.putData( 'Algae', sysAlgae )
        # SmartDashboard.putData( 'Elevator', sysElevator )
        # SmartDashboard.putData( 'Climber', sysClimber )
        #SmartDashboard.putData( 'ReefScape', ReefScapeState )

        ## Driver Controller
        driver1 = FalconXboxController( 0, squaredInputs=ntproperty("/Settings/Driver1/SquaredInputs", True) )
        driver2 = FalconXboxController( 1 )

        ## Commands
        # Coral
        cmdControlPivotPosition = ControlPivotPosition( sysCoralManipulatorPivot, lambda: driver1.getLeftUpDown())
        cmdRunCoralWheel = CoralWheelOpenControl( sysCoralManipulatorWheel, driver1.getRightUpDown )
        cmdSetPivotPositionL1 = SetPivotPosition( sysCoralManipulatorPivot, CoralPivotPositions.L1, 'Trough' )
        cmdSetPivotPositionMAX = SetPivotPosition( sysCoralManipulatorPivot, CoralPivotPositions.MAX, 'Up' )

        # Algae
        # cmdAlgaeGrab = AlgaeGrabCommand( sysAlgae )
        # cmdAlgaeDefault = AlgaeHoldCommand( sysAlgae )
        # cmdAlgaeEject = AlgaeEjectCommand( sysAlgae )

        # Elevator
        # cmdElevatorTo0 = ElevatorToPos(sysElevator, lambda: 0)
        # cmdElevatorTo10 = ElevatorToPos(sysElevator, lambda: 10)
        # cmdElevatorByStick = ElevatorByStick(sysElevator, lambda: driver2.getLeftUpDown())

        # Climber
        # cmdClimberStay = ClimberStay( sysClimber )

        # Drive
        # cmdDriveByStick = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )
        # cmdAwaitVisionData = AwaitVisionData( lambda: sysVision.has_recieved_data, sysDriveTrain.resetOdometry, sysVision.get_last_pose )
        # cmdFollowPathSelect = FollowPathSelect( sysDriveTrain )
        # cmdGetCoral = GetCoral(sysCoralManipulatorWheel, sysCoralManipulatorPivot, sysElevator, sysDriveTrain)
        # cmdToReef = ToReef(sysCoralManipulatorWheel, sysCoralManipulatorPivot, sysElevator, sysDriveTrain)

        ## Default Commands
        #sysCoralManipulatorPivot.setDefaultCommand( cmdControlPivotPosition )
        #sysCoralManipulatorWheel.setDefaultCommand( cmdRunCoralWheel )
        # sysAlgae.setDefaultCommand( cmdAlgaeDefault )
        # sysDriveTrain.setDefaultCommand( cmdDriveByStick )
        # sysClimber.setDefaultCommand( cmdClimberStay )

        # sysElevator.setDefaultCommand( cmdElevatorByStick )

        #cmdAwaitVisionData.schedule()

        driver1.a().toggleOnTrue( SetPivotPosition( sysCoralManipulatorPivot, CoralPivotPositions.START, 'Start' ) )
        driver1.b().toggleOnTrue( SetPivotPosition( sysCoralManipulatorPivot, CoralPivotPositions.L1, 'L1' ) )
        driver1.x().toggleOnTrue( SetPivotPosition( sysCoralManipulatorPivot, CoralPivotPositions.L3, 'L3' ) )
        driver1.y().toggleOnTrue( SetPivotPosition( sysCoralManipulatorPivot, CoralPivotPositions.L4_up, 'L4' ) )

        ## PathPlanner Setup
        # Register Named Commands
        # NamedCommands.registerCommand('Pickup', cmd.waitSeconds(0.25) )
        # NamedCommands.registerCommand('LaunchSpeaker', cmd.waitSeconds(0.25) )

        # Autonomous Chooser
        # self.__autoChooser = AutoBuilder.buildAutoChooser( "Auto Practice" )
        # SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )


    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
