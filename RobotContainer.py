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
        driver1 = FalconXboxController(0, squaredInputs=True)
        driver2 = FalconXboxController(1, squaredInputs=True)

        ## Initialize Subsystems
        sysCoralWheel = CoralManipulatorWheel( 8 )
        sysCoralPivot = CoralManipulatorPivot( 7, 0.7794115 )
        sysClimber = Climber(3, 4, 0.9024424)
        sysElevator = Elevator( 5, 6, driver1.getRightUpDown )
        sysAlgae = AlgaeManipulator()
        sysDriveTrain = SwerveDrive()

        ## Initialize Control Scheme

        match control_mode:
            case "Comp":
                self.__compControls(driver1, driver2, sysCoralWheel, sysCoralPivot, sysClimber, sysAlgae, sysDriveTrain, sysElevator)
            case "Practice":
                self.__practiceControls(driver1, driver2, sysCoralWheel, sysCoralPivot, sysClimber, sysAlgae, sysDriveTrain, sysElevator)
            case "Test":
                self.__testControls(driver1, driver2, sysCoralWheel, sysCoralPivot, sysClimber, sysAlgae, sysDriveTrain, sysElevator)


    # Get Autonomous Command
    def getAutonomousCommand(self) -> Command:
        """
        Get the Autonomous Command that is currently selected in the AutoChooser Dropdown on the Shuffleboard / SmartDashboards
        """
        chooserValue = self.__autoChooser.getSelected()
        return chooserValue if isinstance( chooserValue, Command ) else cmd.none()

    def __compControls(self, driver1, driver2, sysCoralWheel, sysCoralPivot, sysClimber, sysAlgae, sysDriveTrain,
                       sysElevator):
        pass

    def __practiceControls(self, driver1, driver2, sysCoralWheel, sysCoralPivot, sysClimber, sysAlgae, sysDriveTrain,
                           sysElevator):
        pass

    def __testControls(self, driver1, driver2, sysCoralWheel, sysCoralPivot, sysClimber, sysAlgae, sysDriveTrain,
                       sysElevator):
        ### Commands
        ## Coral
        cmdSetPivotl1 = SetPivotPosition(sysCoralPivot, sysCoralPivot.PivotPositions.L1, "SetPivotL1")
        cmdSetPivotL2 = SetPivotPosition(sysCoralPivot, sysCoralPivot.PivotPositions.L2, "SetPivotL2")
        cmdSetPivotL3 = SetPivotPosition(sysCoralPivot, sysCoralPivot.PivotPositions.L3, "SetPivotL3")

        cmdCoralIn = CoralWheelIn(sysCoralWheel)
        cmdCoralOut = CoralWheelOut(sysCoralWheel)
        cmdCoralPivotControl = ControlPivotPosition(sysCoralPivot, driver1.getRightUpDown)

        # Climber
        # cmdClimberPosition = ClimberPosition( sysClimber, driver2.getLeftUpDown )
        cmdClimberStay = ClimberStay( sysClimber )

        # Elevator
        # cmdElevatorByStick = ElevatorByStick( sysElevator, driver1.getRightUpDown )

        # DriveTrain
        cmdDriveByStick = DriveByStick( sysDriveTrain, driver1.getLeftUpDown, driver1.getLeftSideToSide, driver1.getRightSideToSide )

        # Algae
        cmdAlgaeGrab = AlgaeGrab( sysAlgae )
        cmdAlgaeEject = AlgaeEject( sysAlgae )
        cmdAlgaeHold = AlgaeHold( sysAlgae )

        # default commands
        # defaults
        sysCoralPivot.setDefaultCommand(cmdCoralPivotControl)
        # sysClimber.setDefaultCommand(cmdClimberStay)
        sysAlgae.setDefaultCommand(cmdAlgaeHold)
        sysDriveTrain.setDefaultCommand(cmdDriveByStick)

        ### Controls

        ## Coral
        driver1.x().onTrue(cmdCoralIn)
        driver1.y().onTrue(cmdCoralOut)
        # driver1.y().toggleOnTrue(cmdCoralIO)


        ## CLimber
        # driver2.y().toggleOnTrue(cmdClimberPosition)
        # Only necessary climber commands (as of now): ClimberClimb (climber goes in), ClimberNotClimb (climber goes out), ClimberAway (climber goes straight up)
        # driver1.a().onTrue(cmdClimberClimb) # z on keyboard
        # driver1.x().onTrue(cmdClimberNotClimb) # c on keyboard

        ## Elevator
        # driver1.a().toggleOnTrue( cmdElevatorByStick )

        ## Algae
        driver1.a().whileTrue(cmdAlgaeGrab)
        driver1.b().whileTrue(cmdAlgaeEject)
