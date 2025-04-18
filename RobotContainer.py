# FRC Imports
from wpilib import SendableChooser, SmartDashboard
from commands2 import Command, cmd, ConditionalCommand, ParallelDeadlineGroup

from wpimath.geometry import Rotation2d
from wpimath.units import *

from ntcore.util import ntproperty

# Local Imports
from subsystems import *
from commands import *
from sequences import *
from util import FalconXboxController, ReefScape, ReefScapeController, SourceSelect, ReefSide

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
        ## Controller Mapping Mode
        control_mode: str = "Comp"  # Can be "Comp", "Practice", "Test", "DriveOnly"


        ## Controllers
        self.driver1 = FalconXboxController(0, squaredInputs=True)
        self.driver2 = FalconXboxController(1, squaredInputs=True)
        self.controlBoard = ReefScapeController( 2, 3 )

        ## Initialize Subsystems
        self.sysDriveTrain = SwerveDrive()
        self.sysVision     = Vision( self.sysDriveTrain.apply_vision_measurement )
        self.sysElevator   = Elevator( 5, 6 )
        self.sysCoralWheel = CoralManipulatorWheel( 8 )
        self.sysCoralPivot = CoralManipulatorPivot( 7, 0.967803 )
        self.sysAlgae      = AlgaeManipulator()
        self.sysClimber    = Climber( 3, 4, 0.7306183 )
        # self.sysClimber    = ClimberSimple( 3, 4, 0.9136312 )

        # Vision Prep
        AwaitVisionData( self.sysVision, self.sysDriveTrain ).schedule()

        self.sysCoralPivot.setHasCoral(self.sysCoralWheel.hasCoral)

        ## Initialize State
        self.ReefScapeState = ReefScape.getInstance()
        self.ReefScapeState.setHasAlgae( self.sysAlgae.hasAlgae )
        self.ReefScapeState.setHasCoral( self.sysCoralWheel.hasCoral )
        self.ReefScapeState.setGetPose( self.sysDriveTrain.getPose )

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

        ## Initialize Named Commands
        self.__InitNamedCommands()

        ## Initialize Auto Chooser
        self.__autoChooser = AutoBuilder.buildAutoChooser("")
        SmartDashboard.putData("AutoChooser", self.__autoChooser)

    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()

    def __bindCompetitionControls(self):
        """ Console """
        # Reef
        self.controlBoard.R1().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R1 ) ).ignoringDisable(True) )
        self.controlBoard.R2().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R2 ) ).ignoringDisable(True) )
        self.controlBoard.R3().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R3 ) ).ignoringDisable(True) )
        self.controlBoard.R4().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R4 ) ).ignoringDisable(True) )
        self.controlBoard.R5().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R5 ) ).ignoringDisable(True) )
        self.controlBoard.R6().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R6 ) ).ignoringDisable(True) )
        self.controlBoard.R7().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R7 ) ).ignoringDisable(True) )
        self.controlBoard.R8().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R8 ) ).ignoringDisable(True) )
        self.controlBoard.R9().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R9 ) ).ignoringDisable(True) )
        self.controlBoard.R10().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R10 ) ).ignoringDisable(True) )
        self.controlBoard.R11().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R11 ) ).ignoringDisable(True) )
        self.controlBoard.R12().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R12 ) ).ignoringDisable(True) )

        # Source
        self.controlBoard.Inner().onTrue( cmd.runOnce(self.ReefScapeState.setSourceSelect(SourceSelect.INNER)))
        self.controlBoard.Middle().onTrue( cmd.runOnce(self.ReefScapeState.setSourceSelect(SourceSelect.MIDDLE)))
        self.controlBoard.Outer().onTrue( cmd.runOnce(self.ReefScapeState.setSourceSelect(SourceSelect.OUTER)))

        self.controlBoard.Reset().onTrue( cmd.runOnce(self.ReefScapeState.changeSourceSide()))

        """Driver 1"""
        # Drive
        self.sysDriveTrain.setDefaultCommand(
            DriveByStick( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide )
        )
        self.driver1.rightTrigger().whileTrue( DriveToPose( self.sysDriveTrain, self.ReefScapeState.getCoralPose) ) # TODO: Make better and more consistent, use sequences
        self.driver1.leftTrigger().whileTrue( DriveToPose( self.sysDriveTrain, self.ReefScapeState.getReefPose) )
        self.driver1.rightBumper().toggleOnTrue( DriveByStickRotate( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide, ReefScape.getTargetRotation ) )
        self.driver1.leftBumper().onTrue( cmd.runOnce( self.sysDriveTrain.changeDriveSpeedPercent ) )
        self.driver1.back().onTrue( AwaitVisionData( self.sysVision, self.sysDriveTrain ) )
        self.driver1.leftStick().onTrue( cmd.runOnce( self.sysDriveTrain.toggleFieldRelative ) )

        # Algae
        self.sysAlgae.setDefaultCommand( AlgaeHold( self.sysAlgae ) )
        self.driver1.a().whileTrue( AlgaeGrab( self.sysAlgae ) )
        self.driver1.b().whileTrue( AlgaeEject( self.sysAlgae ) )

        """Driver 2"""
        ## Sequences
        self.driver2.a().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.SOURCE, 'Source') ).toggleOnTrue(CoralIO(self.sysCoralWheel))
        self.driver2.a().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.BOTTOM)) # Source button, also moves pivot

        ## Coral
        # Pivot
        self.sysCoralPivot.setDefaultCommand( ControlPivotPosition( self.sysCoralPivot, self.driver2.getRightUpDown ) )
        self.controlBoard.L1().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L1, "L1" ) )
        self.controlBoard.L2().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L2, "L2" ) )
        self.controlBoard.L3().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L3, "L3" ) )
        self.controlBoard.L4().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_up, "L4u" ) )

        # Wheel
        self.sysCoralWheel.setDefaultCommand( CoralDefault( self.sysCoralWheel ) )
        self.driver2.y().toggleOnTrue( CoralIO( self.sysCoralWheel ) )

        # Elevator
        self.sysElevator.setDefaultCommand(ElevatorByStick(self.sysElevator, self.driver2.getLeftUpDown))
        self.controlBoard.L1().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.L1))
        self.controlBoard.L2().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.L2))
        self.controlBoard.L3().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.L3))
        self.controlBoard.L4().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.L4))
        # self.controlBoard.Reset().onTrue(ElevatorResync(self.sysElevator, self.sysCoralPivot))


        # self.controlBoard.Reset().whileTrue( DriveToPose(self.ReefScapeState.getReefPose) )

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
        self.sysClimber.setDefaultCommand( ClimberOpenLoopControl( self.sysClimber, self.driver1.getTriggers ) )

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
        self.driver2.b().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.L2 ) )
        self.driver2.x().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.L3 ) )
        self.driver2.y().toggleOnTrue( ElevatorToPos( self.sysElevator, ElevatorPositions.L4 ) )

    def __bindTestControls(self):

        """ Console """
        self.controlBoard.R1().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R1 ) ).ignoringDisable(True) )
        self.controlBoard.R2().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R2 ) ).ignoringDisable(True) )
        self.controlBoard.R3().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R3 ) ).ignoringDisable(True) )
        self.controlBoard.R4().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R4 ) ).ignoringDisable(True) )
        self.controlBoard.R5().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R5 ) ).ignoringDisable(True) )
        self.controlBoard.R6().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R6 ) ).ignoringDisable(True) )
        self.controlBoard.R7().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R7 ) ).ignoringDisable(True) )
        self.controlBoard.R8().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R8 ) ).ignoringDisable(True) )
        self.controlBoard.R9().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R9 ) ).ignoringDisable(True) )
        self.controlBoard.R10().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R10 ) ).ignoringDisable(True) )
        self.controlBoard.R11().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R11 ) ).ignoringDisable(True) )
        self.controlBoard.R12().onTrue( cmd.runOnce( lambda: self.ReefScapeState.setTarget( ReefSide.R12 ) ).ignoringDisable(True) )

        """Driver 1"""
        # Drive
        self.sysDriveTrain.setDefaultCommand(
            DriveByStick( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide )
        )
        
        #self.driver1.rightBumper().whileTrue( DriveToPose(self.ReefScapeState.getReefPose) ) # TODO: Make better and more consistent, use sequences
        self.driver1.rightBumper().toggleOnTrue( DriveByStickRotate( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide, ReefScape.getTargetRotation ) )
        self.driver1.leftBumper().onTrue( cmd.runOnce( self.sysDriveTrain.changeDriveSpeedPercent ) )
        self.driver1.back().onTrue( AwaitVisionData( self.sysVision, self.sysDriveTrain ) )
        self.driver1.start().onTrue( cmd.runOnce( self.sysDriveTrain.toggleFieldRelative ) )
        self.driver1.leftStick().onTrue( cmd.runOnce( self.sysDriveTrain.toggleFieldRelative ) )

        # Climber
        # self.driver1.y().toggleOnTrue( ClimberOpenLoopControl( self.sysClimber, self.driver1.getTriggers ) ).toggleOnTrue( AlgaeOut( self.sysAlgae ) )

        # Algae
        self.sysAlgae.setDefaultCommand( AlgaeHold( self.sysAlgae ) )
        self.driver1.a().whileTrue( AlgaeGrab( self.sysAlgae ) )
        self.driver1.b().whileTrue( AlgaeEject( self.sysAlgae ) )

        """Driver 2"""
        ## Coral
        # Pivot
        self.sysCoralPivot.setDefaultCommand( ControlPivotPosition( self.sysCoralPivot, self.driver2.getRightUpDown ) )
        self.controlBoard.L1().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L1, "L1" ) )
        self.controlBoard.L2().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L2, "L2" ) )
        self.controlBoard.L3().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L3, "L3" ) )
        self.controlBoard.L4().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_up, "L4u" ) )
        SmartDashboard.putData('PivotL1', SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L1, "L1" ))
        SmartDashboard.putData('PivotL2', SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L2, "L2" ))
        SmartDashboard.putData('PivotL3', SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L3, "L3" ))
        SmartDashboard.putData('PivotL4u', SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_up, "L4u" ))
        # self.controlBoard.L1().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.L4_down, "L4d" ) )
        # self.driver2.b().toggleOnTrue( ControlPivotPosition( self.sysCoralPivot, self.driver2.getRightUpDown ) )

        self.driver2.a().toggleOnTrue( SetPivotPosition( self.sysCoralPivot, CoralPivotPositions.SOURCE, 'Source') ).toggleOnTrue(CoralWheelIn(self.sysCoralWheel))

        # Wheel
        self.sysCoralWheel.setDefaultCommand( CoralDefault( self.sysCoralWheel ) )
        self.driver2.y().toggleOnTrue( CoralIO( self.sysCoralWheel ) )

        # Source
        self.controlBoard.Inner().onTrue( cmd.runOnce(self.ReefScapeState.setSourceSelect(SourceSelect.INNER)))
        self.controlBoard.Middle().onTrue( cmd.runOnce(self.ReefScapeState.setSourceSelect(SourceSelect.MIDDLE)))
        self.controlBoard.Outer().onTrue( cmd.runOnce(self.ReefScapeState.setSourceSelect(SourceSelect.OUTER)))

        self.controlBoard.Reset().onTrue( cmd.runOnce(self.ReefScapeState.changeSourceSide()))


        # Elevator
        self.sysElevator.setDefaultCommand(ElevatorByStick(self.sysElevator, self.driver2.getLeftUpDown))
        self.controlBoard.L1().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.L1))
        self.controlBoard.L2().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.L2))
        self.controlBoard.L3().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.L3))
        self.controlBoard.L4().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.L4))
        self.controlBoard.Reset().onTrue(ElevatorResync(self.sysElevator, self.sysCoralPivot))
        SmartDashboard.putData('ElevatorL1', ElevatorToPos(self.sysElevator, ElevatorPositions.L1))
        SmartDashboard.putData('ElevatorL2', ElevatorToPos(self.sysElevator, ElevatorPositions.L2))
        SmartDashboard.putData('ElevatorL3', ElevatorToPos(self.sysElevator, ElevatorPositions.L3))
        SmartDashboard.putData('ElevatorL4', ElevatorToPos(self.sysElevator, ElevatorPositions.L4))

        self.driver2.a().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.SOURCE)) # Source button, also moves pivot
        self.driver2.x().onTrue(ElevatorScoreL4(self.sysElevator, self.sysCoralWheel))

        # self.controlBoard.Reset().whileTrue( DriveToPose(self.ReefScapeState.getReefPose) )

        # DriveTrain        
        # def toggleDrive():
        #     assert self.sysDriveTrain.getDefaultCommand() is not None

        #     if self.sysDriveTrain.getDefaultCommand().getName() == "DriveByStick":
        #         self.sysDriveTrain.setDefaultCommand( autoRotStick )
        #     else:
        #         self.sysDriveTrain.setDefaultCommand( normalStick )

        #     self.sysDriveTrain.getCurrentCommand().cancel()

        # normalStick = DriveByStick( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide )
        # autoRotStick = DriveByStickRotate( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide, ReefScape.getTargetRotation )

        # self.sysDriveTrain.setDefaultCommand( normalStick )
        # #self.driver1.a().onTrue( cmd.runOnce( toggleDrive ) )
        # self.driver1.a().onTrue( autoRotStick )

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
        # # self.sysCoralPivot.setDefaultCommand()
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

        # self.sysCoralPivot.setDefaultCommand( ControlPivotPosition( self.sysCoralPivot, self.driver2.getLeftUpDown ) )

        # self.sysElevator.setDefaultCommand(ElevatorByStick(self.sysElevator, self.driver1.getRightUpDown))
        # self.driver1.a().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.TROUGH))
        # self.driver1.b().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.LOW_CORAL))
        # self.driver1.x().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.MED_CORAL))
        # self.driver1.y().onTrue(ElevatorToPos(self.sysElevator, ElevatorPositions.HIGH_CORAL))

        # self.sysClimber.setDefaultCommand( ClimberOpenLoopControl( self.sysClimber, self.driver1.getLeftUpDown ) )
        # # DO NOT USE THIS # self.driver1.a().toggleOnTrue( ClimberOut( self.sysClimber ) )


        # self.driver1.a().toggleOnTrue( AlgaeGrab( self.sysAlgae ) )
        # self.driver1.b().toggleOnTrue( AlgaeHold( self.sysAlgae ) )
        # self.driver1.x().toggleOnTrue( cmd.none() )

    def __bindDriveOnly(self):
        self.sysDriveTrain.setDefaultCommand(
            DriveByStick( self.sysDriveTrain, self.driver1.getLeftUpDown, self.driver1.getLeftSideToSide, self.driver1.getRightSideToSide )
        )

        self.driver1.a().onTrue( cmd.runOnce(self.sysDriveTrain.resetOdometry) )


    def __InitNamedCommands(self):
        """
        Initialize Named Commands for PathPlanner
        """
        NamedCommands.registerCommand("L4 Elevator", ElevatorToPos(self.sysElevator, ElevatorPositions.L4))
        NamedCommands.registerCommand("L3 Elevator", ElevatorToPos(self.sysElevator, ElevatorPositions.L3))
        NamedCommands.registerCommand("L2 Elevator", ElevatorToPos(self.sysElevator, ElevatorPositions.L2))
        NamedCommands.registerCommand("L1 Elevator", ElevatorToPos(self.sysElevator, ElevatorPositions.L1))
        NamedCommands.registerCommand("MIN Elevator", ElevatorToPos(self.sysElevator, ElevatorPositions.BOTTOM))

        NamedCommands.registerCommand("L3.5 Elevator", ElevatorToPos( self.sysElevator, ElevatorPositions.L35 ))
        NamedCommands.registerCommand("L2.5 Elevator", ElevatorToPos( self.sysElevator, ElevatorPositions.L25 ))

        NamedCommands.registerCommand("L4 Pivot", SetPivotPosition(self.sysCoralPivot, CoralPivotPositions.L4_up, "L4u"))
        NamedCommands.registerCommand("L3 Pivot", SetPivotPosition(self.sysCoralPivot, CoralPivotPositions.L3, "L3"))
        NamedCommands.registerCommand("L2 Pivot", SetPivotPosition(self.sysCoralPivot, CoralPivotPositions.L2, "L2"))
        NamedCommands.registerCommand("L1 Pivot", SetPivotPosition(self.sysCoralPivot, CoralPivotPositions.L1, "L1"))
        NamedCommands.registerCommand("MIN Pivot", SetPivotPosition(self.sysCoralPivot, CoralPivotPositions.MIN, "L0"))

        NamedCommands.registerCommand("Coral Out", CoralWheelOut(self.sysCoralWheel))
        NamedCommands.registerCommand("Wait For Pickup", CoralWheelIn(self.sysCoralWheel))
        NamedCommands.registerCommand("Coral Hold", CoralHoldAuto( self.sysCoralWheel ))