import typing, threading
import math

from commands2 import Subsystem
from wpilib import RobotState, SmartDashboard, Field2d, DriverStation
from wpilib.shuffleboard import Shuffleboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Translation2d, Pose2d, Pose3d, Transform2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition, SwerveModuleState, SwerveDrive4Odometry, ChassisSpeeds
from wpimath.system.plant import DCMotor
from wpimath.units import lbsToKilograms

from ntcore.util import ntproperty

from phoenix6.hardware import Pigeon2

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants, ModuleConfig
from pathplannerlib.logging import PathPlannerLogging

from subsystems.Swerve.SwerveModule import SwerveModule, SwerveModuleConstants
from util import FalconLogger

class SwerveDriveConstants:
    kWeightLbs = 120.0
    kMaxSpeed = 4.4
    kMaxRotationSpeed = math.pi*3.6

class SwerveDrive(Subsystem):
    # Variable Declaration
    __modules:typing.Tuple[ SwerveModule, SwerveModule, SwerveModule, SwerveModule ] = None
    __gyro:Pigeon2 = None
    __kinematics:SwerveDrive4Kinematics = None
    __odometry:SwerveDrive4Odometry = None
    __visionOdometry:SwerveDrive4PoseEstimator = None

    # Settings
    __DriveFieldRelative = ntproperty( "/Settings/Driver1/FieldRelative", True )
    __DriveMaxSpeedPercent = ntproperty( "/Settings/Driver1/MaxSpeedPercent", 0.75 )
    __DriveMaxRotationPercent = ntproperty( "/Settings/Driver1/MaxRotationPercent", 0.36 )

    __setpoint:ChassisSpeeds = None
    __setpointStates = [ SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState() ]
    # __odometryLock = False

    # Initialization
    def __init__(self) -> None:
        self.setName( "SwerveDrive" )

        self.__gyro = Pigeon2( 0, "canivore1" )

        self.__modules = [
            SwerveModule( 0, 1, 2, 1, -0.329590 ), # 97.471 ),
            SwerveModule( 1, 3, 4, 2, 0.249023 ), #5.361 ),
            SwerveModule( 2, 5, 6, 3, -0.052979 ), #-0.104004 ), #298.828 ),325439
            SwerveModule( 3, 7, 8, 4, 0.423584 ) #60.557 )0.334473
        ]

        self.__kinematics = SwerveDrive4Kinematics(
            Translation2d( 0.27305, 0.27305 ),
            Translation2d( 0.27305, -0.27305 ),
            Translation2d( -0.27305, 0.27305 ),
            Translation2d( -0.27305, -0.27305 )
        )

        self.__odometry = SwerveDrive4Odometry(
            self.__kinematics,
            self.__gyro.getRotation2d(),
            self.__getModulePositions(),
            Pose2d(8.7, 4, Rotation2d(0)) # This is where we would set an initial pose on the field
        )
        self.__visionOdometry = SwerveDrive4PoseEstimator(
            self.__kinematics,
            self.__gyro.getRotation2d(),
            self.__getModulePositions(),
            Pose2d(8.7, 4, Rotation2d(0)) # This is where we would set an initial pose on the field
        )

        self.stop()

        # Dashboards
        self.__field = Field2d()
        SmartDashboard.putData("Field", self.__field)
       
        # Path Planner
        robotConfig = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            pose_supplier = self.__visionOdometry.getEstimatedPosition,
            reset_pose = self.__visionOdometry.resetPose,
            robot_relative_speeds_supplier = self.getChassisSpeeds,
            output = lambda speeds, feedforwards: self.runChassisSpeeds(speeds),
            controller = PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
            ),
            robot_config = robotConfig,
            should_flip_path = self.shouldFlipPath,
            drive_subsystem = self
        )

        # PathPlannerLogging.setLogCurrentPoseCallback( self.__field.setRobotPose )
        # PathPlannerLogging.setLogTargetPoseCallback( self.__field.getObject('targetPose').setPose )
        # PathPlannerLogging.setLogActivePathCallback( self.__field.getObject('path').setPoses )
        SmartDashboard.putData( 'Drive', self )

    def shouldFlipPath(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    # def resetGyro(self) -> None:
    #     '''
    #     sets the current gyroscope yaw offset to the estimated position from vision data\n
    #     uses threading, there is no 100ms timeout
    #     '''
    #     # Thread Safe Function
    #     def resetGyroThread() -> None:
    #         # Get The Current Pose Data
    #         currentPose = self.__visionOdometry.getEstimatedPosition()
    #         self.__gyro.set_yaw( currentPose.rotation().degrees(), 0.3 )
    #         self.__resetOdometry( currentPose )
    #         self.__odometryLock = False
    #         print( "OdometryUnlocked!" )

    #     # Odometry Lock
    #     self.__odometryLock = True
    #     print( "OdometryLock!" )
    #     threading.Thread( target=lambda: resetGyroThread() ).start()

    def resetOdometry(self, pose:Pose2d|None=None) -> None:
        self.__odometry.resetPosition(self.__gyro.getRotation2d(),
                                      self.__getModulePositions(),
                                      pose if pose else self.__visionOdometry.getEstimatedPosition() )
        self.__visionOdometry.resetPosition(self.__gyro.getRotation2d(),
                                            self.__getModulePositions(),
                                            pose if pose else self.__visionOdometry.getEstimatedPosition() )

    def periodic(self) -> None:
        ## Input Logging
        FalconLogger.logInput( "SwerveDrive/Gyro/yaw_d", self.__gyro.get_yaw().value )
        FalconLogger.logInput( "SwerveDrive/Gyro/pitch_d", self.__gyro.get_pitch().value  )
        FalconLogger.logInput( "SwerveDrive/Gyro/roll_d", self.__gyro.get_roll().value  )
        # FalconLogger.logInput( "SwerveDrive/OdometryLock", self.__odometryLock )

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        
        ## Run SwerveModules
        self.__modules[0].run()
        self.__modules[1].run()
        self.__modules[2].run()
        self.__modules[3].run()
        
        ## Update Odometry
        pose = self.__odometry.getPose()
        vPose = self.__visionOdometry.getEstimatedPosition()

        # if not self.__odometryLock:
        pose = self.__odometry.update(
            self.__gyro.getRotation2d(),
            self.__getModulePositions()
        )

        # Update Vision Odometry
        vPose = self.__visionOdometry.update(
            self.__gyro.getRotation2d(),
            self.__getModulePositions()
        )
        
        ## Dashboarding -- Updated Odometry to only use Blue Relative
        self.__field.setRobotPose( pose )
        self.__field.getObject( "BlueVisionPose" ).setPose( vPose )
       
        ## Output Logging
        FalconLogger.logOutput( "SwerveDrive/Odometry", pose )
        FalconLogger.logOutput( "SwerveDrive/OdometryPlusVision", vPose )
        FalconLogger.logOutput( "SwerveDrive/ChassisSpeeds/Actual", self.getChassisSpeeds() )
        FalconLogger.logOutput( "SwerveDrive/ChassisSpeeds/Target", self.__setpoint )
        FalconLogger.logOutput( "SwerveDrive/SwerveModuleStates/Actual", self.__getModuleStates() )
        FalconLogger.logOutput( "SwerveDrive/SwerveModuleStates/Target", self.__getDesiredModuleStates() )

    def simulationPeriodic(self) -> None:
        # Run SwerveModules
        self.__modules[0].runSim()
        self.__modules[1].runSim()
        self.__modules[2].runSim()
        self.__modules[3].runSim()

        # Update Gyro
        moduleStates = self.__getModuleStates()
        actualSpeed = self.__kinematics.toChassisSpeeds( moduleStates )
        yawPer20ms = actualSpeed.omega_dps * 0.02
        self.__gyro.sim_state.add_yaw( yawPer20ms )

    # Stop the Subsystem
    def stop(self) -> None:
        self.runChassisSpeeds( ChassisSpeeds( 0.0, 0.0, 0.0 ) )
 
    def getRobotAngle(self) -> Rotation2d:
        return self.__visionOdometry.getEstimatedPosition().rotation().rotateBy(
            Rotation2d.fromDegrees(180.0 if self.shouldFlipPath() else 0.0)
        )

    # Run By Percentage
    def runPercentInputs(self, x:float, y:float, omega:float) -> None:
        # Range Tolerances
        x = min( max( x, -1.0 ), 1.0 )
        y = min( max( y, -1.0 ), 1.0 )
        omega = min( max( omega, -1.0 ), 1.0 )

        xSpeed = x * self.__DriveMaxSpeedPercent * SwerveDriveConstants.kMaxSpeed
        ySpeed = y * self.__DriveMaxSpeedPercent * SwerveDriveConstants.kMaxSpeed
        omegaSpeed = omega * self.__DriveMaxRotationPercent * SwerveDriveConstants.kMaxRotationSpeed

        cSpeed = (
            ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, omegaSpeed, self.getRobotAngle() )
            if self.__DriveFieldRelative
            else ChassisSpeeds( xSpeed, ySpeed, omegaSpeed )
        )
        self.runChassisSpeeds( cSpeed )

    # Run By Chassis Speeds
    def runChassisSpeeds(self, chassisSpeeds:ChassisSpeeds) -> None:
        self.__setpoint = ChassisSpeeds.discretize( chassisSpeeds, 0.02 )
        self.__setpointStates = self.__kinematics.toSwerveModuleStates( self.__setpoint, Translation2d(0,0) )
        self.runModuleStates( self.__setpointStates )

    # Run By SwerveModuleStates
    def runModuleStates(self, swerveStates:typing.Tuple[ SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState ]) -> None:
        newStates = SwerveDrive4Kinematics.desaturateWheelSpeeds( swerveStates, SwerveDriveConstants.kMaxSpeed )
        self.__modules[0].setState(newStates[0])
        self.__modules[1].setState(newStates[1])
        self.__modules[2].setState(newStates[2])
        self.__modules[3].setState(newStates[3])

    def getOdometry(self) -> SwerveDrive4PoseEstimator:
        # if self.__odometryLock:
        #     raise "Odometry Lock In Place" 
        return self.__visionOdometry

    def __getModulePositions(self) -> typing.Tuple[ SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        return [
            self.__modules[0].getPosition(),
            self.__modules[1].getPosition(),
            self.__modules[2].getPosition(),
            self.__modules[3].getPosition()
        ]
    
    def __getModuleStates(self) -> typing.Tuple[ SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return [
            self.__modules[0].getState(),
            self.__modules[1].getState(),
            self.__modules[2].getState(),
            self.__modules[3].getState()
        ]
    
    def __getDesiredModuleStates(self) -> typing.Tuple[ SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return [
            self.__modules[0].getDesiredState(),
            self.__modules[1].getDesiredState(),
            self.__modules[2].getDesiredState(),
            self.__modules[3].getDesiredState()
        ]

    def getChassisSpeeds(self) -> ChassisSpeeds:
        return self.__kinematics.toChassisSpeeds( self.__getModuleStates() )
    
    def getPose(self) -> Pose2d:
        return self.__visionOdometry.getEstimatedPosition()

    def getMaxSpeed(self) -> float:
        return SwerveDriveConstants.kMaxSpeed
    
    def getMaxRotation(self) -> float:
        return SwerveDriveConstants.kMaxRotationSpeed

    def getDriverMaxSpeed(self) -> float:
        return SwerveDriveConstants.kMaxSpeed * self.__DriveMaxSpeedPercent
    
    def getDriverMaxRotation(self) -> float:
        return SwerveDriveConstants.kMaxRotationSpeed * self.__DriveMaxRotationPercent

    def getFieldRelative(self) -> bool:
        return self.__DriveFieldRelative
    
    def setFieldRelative(self, isFieldRelative:bool) -> None:
        self.__DriveFieldRelative = isFieldRelative

    def toggleFieldRelative(self) -> None:
        self.__DriveFieldRelative = not self.__DriveFieldRelative
    
    def changeDriveSpeedPercent(self) -> None:
        if self.__DriveMaxSpeedPercent == 0.75:
            self.__DriveMaxSpeedPercent = 0.35
        elif self.__DriveMaxSpeedPercent == 0.35:
            self.__DriveMaxSpeedPercent = 0.75
    # def setRotSpeedPercent(self, val:float) -> None:
    #     self.__DriveMaxSpeedPercent = val