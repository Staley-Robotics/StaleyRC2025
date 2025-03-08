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

        ## Configure State
        ReefScapeState = ReefScape.getInstance()
        ReefScapeState.setSourceSide(SourceSide.RIGHT)
        ReefScapeState.setSourceSelect(SourceSelect.OUTER)
        ReefScapeState.setHasCoral(False)

        ## Declare Subsystems
        sysDriveTrain = SwerveDrive()
        sysVision = Vision( sysDriveTrain.getOdometry )

        sysCoralManipulatorPivot = CoralManipulatorPivot( 'CoralManipulatorPivot', 12, 20 )
        sysCoralManipulatorWheel = CoralManipulatorWheel( 'CoralManipulatorWheel', 3 )

        sysAlgae = AlgaeManipulator()
        sysElevator = Elevator()
        sysClimber = Climber( 15 )

        # Put Subsystems on NetworkTables
        SmartDashboard.putData( '/Subsystems/DriveTrain', sysDriveTrain )
        SmartDashboard.putData( '/Subsystems/Vision', sysVision )
        SmartDashboard.putData( '/Subsystems/CoralPivot', sysCoralManipulatorPivot )
        SmartDashboard.putData( '/Subsystems/CoralWheel', sysCoralManipulatorWheel )
        SmartDashboard.putData( '/Subsystems/Algae', sysAlgae )
        SmartDashboard.putData( '/Subsystems/Elevator', sysElevator )
        SmartDashboard.putData( '/Subsystems/Climber', sysClimber )
        SmartDashboard.putData( '/ReefScape', ReefScapeState )


        ## Driver Controller
        driver1 = FalconXboxController( 0, squaredInputs=ntproperty("/Settings/Driver1/SquaredInputs", True) )

        ## Commands
        # Coral
        cmdControlPivotPosition = ControlPivotPosition( sysCoralManipulatorPivot, lambda: driver1.getLeftUpDown())
        cmdRunCoralWheel = CoralWheelOpenControl( sysCoralManipulatorWheel, driver1.getRightUpDown )
        cmdSetPivotPositionL1 = SetPivotPosition( sysCoralManipulatorPivot, CoralManipulatorPivot.PivotPositions.L1, 'Trough' )
        cmdSetPivotPositionMAX = SetPivotPosition( sysCoralManipulatorPivot, CoralManipulatorPivot.PivotPositions.MAX, 'Up' )

        # Algae
        cmdAlgaeGrab = AlgaeGrabCommand( sysAlgae )
        cmdAlgaeDefault = AlgaeHoldCommand( sysAlgae )
        cmdAlgaeEject = AlgaeEjectCommand( sysAlgae )

        # Elevator
        cmdElevatorTo0 = ElevatorToPos(sysElevator, 0)
        cmdElevatorTo10 = ElevatorToPos(sysElevator, 10)
        cmdElevatorByStuck = ElevatorByStick(sysElevator, lambda: driver1.getRightUpDown())

        cmdElevatorTo0 = ElevatorToPos(sysElevator, 0)
        cmdElevatorTo10 = ElevatorToPos(sysElevator, 10)
        cmdElevatorByStuck = ElevatorByStick(sysElevator, lambda: driver1.getRightUpDown())

        # Climber
        cmdClimberStay = ClimberStay( sysClimber )

        # Drive
        cmdDriveByStick = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )
        cmdAwaitVisionData = AwaitVisionData( lambda: sysVision.has_recieved_data, sysDriveTrain.resetOdometry, sysVision.get_last_pose )
        # cmdFollowPathSelect = FollowPathSelect( sysDriveTrain )
        cmdGetCoral = GetCoral(sysCoralManipulatorWheel, sysCoralManipulatorPivot, sysElevator, sysDriveTrain)
        cmdToReef = ToReef(sysCoralManipulatorWheel, sysCoralManipulatorPivot, sysElevator, sysDriveTrain)

        ## Default Commands
        sysCoralManipulatorPivot.setDefaultCommand( cmdControlPivotPosition )
        sysCoralManipulatorWheel.setDefaultCommand( cmdRunCoralWheel )
        sysAlgae.setDefaultCommand( cmdAlgaeDefault )
        sysDriveTrain.setDefaultCommand( cmdDriveByStick )
        sysClimber.setDefaultCommand( cmdClimberStay )

        cmdAwaitVisionData.schedule()

        ## Driver Controller Button Binding
        # driver1.y().whileTrue( cmdAlgaeGrab )
        # driver1.b().whileTrue( cmdAlgaeEject )

        driver1.b().onTrue(cmd.runOnce(lambda: ReefScapeState.changeSourceSide()))
        driver1.y().onTrue(cmd.runOnce(lambda: ReefScapeState.changeSourceSelect()))

        driver1.back().onTrue( cmd.runOnce( sysDriveTrain.resetOdometry() ) )

        # driver1.a().whileTrue(ClimberClimb(sysClimber)) # TODO: move these to be created with other commands
        # driver1.x().whileTrue(ClimberNotClimb(sysClimber))

        driver1.x().whileTrue( cmdGetCoral )
        driver1.a().whileTrue( cmdToReef )

        driver1.pov(0).onTrue(cmd.runOnce(lambda: ReefScape.getInstance().setHasCoral(True)))
        driver1.pov(180).onTrue(cmd.runOnce(lambda: ReefScape.getInstance().setHasCoral(False)))

        driver1.pov(45).onTrue(cmd.runOnce(lambda: ReefScapeState.setNextSide()))
        driver1.pov(90).onTrue(cmd.runOnce(lambda: ReefScapeState.setNextHeight()))

        # driver1.a().onTrue( cmdSetPivotPositionMAX )
        # driver1.b().onTrue( cmdSetPivotPositionL1 )

        # driver1.pov(0).onTrue(cmdElevatorTo0)
        # driver1.pov(90).onTrue(cmdElevatorTo10)
        # driver1.y().whileTrue( cmdElevatorByStuck )

        # driver1.a().onTrue( cmdFollowPathSelect )

        ## PathPlanner Setup
        # Register Named Commands
        # NamedCommands.registerCommand('Pickup', cmd.waitSeconds(0.25) )
        # NamedCommands.registerCommand('LaunchSpeaker', cmd.waitSeconds(0.25) )

        # Autonomous Chooser
        self.__autoChooser = AutoBuilder.buildAutoChooser( "Auto Practice" )
        SmartDashboard.putData( "Autonomous Mode", self.__autoChooser )


    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()
