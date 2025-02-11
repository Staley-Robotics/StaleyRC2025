import typing
import math

from wpilib import RobotBase
from wpilib.shuffleboard import Shuffleboard
# from wpimath.controller import PIDController, ProfiledPIDControllerRadians, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath import applyDeadband
from wpimath.system.plant import DCMotor
from wpimath.units import radiansPerSecondToRotationsPerMinute, rotationsToRadians, radiansToRotations, kSecondsPerMinute, radians, meters_per_second, meters

from rev import SparkMax, SparkRelativeEncoder, SparkMaxSim, SparkMaxConfig

from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration
from phoenix6.controls import VelocityVoltage, PositionVoltage
from phoenix6.signals.spn_enums import SensorDirectionValue, NeutralModeValue, InvertedValue, FeedbackSensorSourceValue

from util import FalconLogger, NTTunableFloat

class SwerveModuleConstants:
    class Drive:
        kWheelRadius:float = 0.0508
        kGearRatio:float = 1 / 6.75
        kP:float = 0 #0.1
        kI:float = 0
        kD:float = 0
        kS:float = 0 #0.1
        kV:float = 0.115#2.8235 #0.13
    
    class Turn:
        kGearRatio:float = 1 / (150/7)
        kP:float = 50#2.5 # 25.0
        kI:float = 0.05
        kD:float = 0
        kMaxAngularVelocity:float = math.pi
        kMaxAngularAcceleration:float = math.pi
        kS:float = 0
        kV:float = 0
    
    class KrakenSim:
        kMaxRps:float = radiansToRotations( DCMotor.krakenX60(1).freeSpeed )# * kSecondsPerMinute

class SwerveModule:
    # Variable Declaration
    __driveMotor:TalonFX = None

    __turnMotor:TalonFX = None
    __turnEncoder:CANcoder = None

    __setpoint:SwerveModuleState = None

    def __init__(self, moduleId:int, driveId:int, turnId:int, encoderId:int, encoderOffset:float):
        ## Susbsystem setup
        self.moduleId = moduleId
        
        ## Drive Motor
        # basic config
        driveMotorCfg = TalonFXConfiguration()
        driveMotorCfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        driveMotorCfg.current_limits.with_stator_current_limit( 40.0 ).with_stator_current_limit_enable( True )
        driveMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # pid config
        driveMotorCfg.slot0.k_p = SwerveModuleConstants.Drive.kP
        driveMotorCfg.slot0.k_i = SwerveModuleConstants.Drive.kI
        driveMotorCfg.slot0.k_d = SwerveModuleConstants.Drive.kD
        driveMotorCfg.slot0.k_v = SwerveModuleConstants.Drive.kV
        self.__driveController = VelocityVoltage(0)
        #motor init
        self.__driveMotor = TalonFX( driveId, "canivore1" )
        self.__driveMotor.configurator.apply( driveMotorCfg )

        ## Turn Encoder (CANcoder)
        # config
        turnEncoderCfg = CANcoderConfiguration()
        turnEncoderCfg.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        turnEncoderCfg.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        if not RobotBase.isSimulation(): turnEncoderCfg.magnet_sensor.magnet_offset = encoderOffset
        # encoder init
        self.__turnEncoder = CANcoder( encoderId, "canivore1" )
        self.__turnEncoder.configurator.apply( turnEncoderCfg )
        self.__turnEncoder.set_position( self.__turnEncoder.get_absolute_position().value ) # Position Safeguard

        ## Turn Motor
        # basic config
        turnMotorCfg = TalonFXConfiguration()
        turnMotorCfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        turnMotorCfg.current_limits.with_stator_current_limit( 20.0 ).with_stator_current_limit_enable( True )
        turnMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        turnMotorCfg.feedback.feedback_remote_sensor_id = encoderId
        turnMotorCfg.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        # pid config
        turnMotorCfg.slot0.k_p = SwerveModuleConstants.Turn.kP
        turnMotorCfg.slot0.k_i = SwerveModuleConstants.Turn.kI
        turnMotorCfg.slot0.k_d = SwerveModuleConstants.Turn.kD
        turnMotorCfg.slot0.k_v = SwerveModuleConstants.Turn.kV
        self.turn_kP = NTTunableFloat('stuff/turnPID/kP', SwerveModuleConstants.Drive.kP, self.updatePID, False)
        self.turn_kI = NTTunableFloat('stuff/turnPID/kI', SwerveModuleConstants.Drive.kI, self.updatePID, False)
        self.turn_kD = NTTunableFloat('stuff/turnPID/kD', SwerveModuleConstants.Drive.kD, self.updatePID, False)
        turnMotorCfg.closed_loop_general.continuous_wrap = True
        self.__turnController = PositionVoltage( 0 )
        # motor init
        self.__turnMotor = TalonFX( turnId, "canivore1" )
        self.__turnMotor.configurator.apply(turnMotorCfg)
        
        # CANcoder Simulation Setup
        self.__turnEncoderSim = self.__turnEncoder.sim_state

        # Default Desired State
        self.__setpoint = SwerveModuleState(0, self.__getTurnEncoderRotation())

        # Dashboards
    
    def updatePID(self):
        config = SparkMaxConfig()
        config.closedLoop.P( self.turn_kP, 0 ).I( self.turn_kI, 0 ).D( self.turn_kD, 0 )
        self.__turnMotor.configurator.apply( config )

    def run(self) -> None:
        # Logging Current State
        logPath = f"SwerveDrive/SwerveModule/{self.moduleId}"
        FalconLogger.logInput( f"{logPath}/DriveInput", self.__driveMotor.get() )
        FalconLogger.logInput( f"{logPath}/DriveOutput", self.__driveMotor.get_motor_voltage().value )
        FalconLogger.logInput( f"{logPath}/DrivePosition_r", self.__driveMotor.get_position().value )
        FalconLogger.logInput( f"{logPath}/DriveVelocity_rpm", self.__driveMotor.get_velocity().value )

        FalconLogger.logInput( f"{logPath}/TurnInput", self.__turnMotor.get() )
        FalconLogger.logInput( f"{logPath}/TurnOutput", self.__turnMotor.get_motor_voltage().value )
        FalconLogger.logInput( f"{logPath}/TurnPosition_r", self.__turnMotor.get_position().value )
        FalconLogger.logInput( f"{logPath}/TurnVelocity_rpm", self.__turnMotor.get_velocity().value )

        FalconLogger.logInput( f"{logPath}/EncoderPositionAbs_r", self.__turnEncoder.get_absolute_position().value )
        FalconLogger.logInput( f"{logPath}/EncoderPositionRel_r", self.__turnEncoder.get_position().value )
        FalconLogger.logInput( f"{logPath}/EncoderVelocity_rps", self.__turnEncoder.get_velocity().value )

        # Set Drive Motor
        desiredMotorVelocity = self.__calcDesiredMotorVelocity( self.__setpoint.speed )
        self.__driveMotor.set_control( self.__driveController.with_velocity( desiredMotorVelocity ) )

        # Set Turn Motor
        self.__turnMotor.set_control( self.__turnController.with_position( radiansToRotations(self.__setpoint.angle.radians()) ) )

    def runSim(self) -> None:
        # Drive Motor Position and Velocity
        driveRps = SwerveModuleConstants.KrakenSim.kMaxRps * self.__driveMotor.get()
        self.__driveMotor.sim_state.set_rotor_velocity(driveRps)
        self.__driveMotor.sim_state.add_rotor_position( driveRps * 0.02 )

        # Turn Motor Position and Velocity
        turnRps = SwerveModuleConstants.KrakenSim.kMaxRps * self.__turnMotor.get()
        self.__turnMotor.sim_state.set_rotor_velocity( turnRps )
        self.__turnMotor.sim_state.add_rotor_position( turnRps * 0.02 )

        # CANcoder Velocity and Position
        canRps = turnRps * SwerveModuleConstants.Turn.kGearRatio
        self.__turnEncoderSim.set_velocity( canRps )
        self.__turnEncoderSim.add_position( canRps * 0.02 )

    def setState(self, desiredState:SwerveModuleState) -> None:
        # Optimize
        currentRotation = self.__getTurnEncoderRotation()
        self.__setpoint = desiredState
        self.__setpoint.optimize( currentRotation )
        self.__setpoint.cosineScale( currentRotation ) # Smooth Out Turning

    def getState(self) -> SwerveModuleState:
        driveVelocity = self.__getDriveWheelVelocity( self.__driveMotor.get_velocity().value )
        return SwerveModuleState(
            driveVelocity,
            self.__getTurnEncoderRotation()
        )
    
    def getDesiredState(self) -> SwerveModuleState:
        return self.__setpoint

    def getPosition(self) -> SwerveModulePosition:
        driveDistance = self.__getDriveDistance( self.__driveMotor.get_position().value )
        return SwerveModulePosition(
            driveDistance,
            self.__getTurnEncoderRotation()
        )

    def __getDriveDistance(self, rotations:float) -> float:
        wheelRotations = rotations * SwerveModuleConstants.Drive.kGearRatio
        wheelRadians = rotationsToRadians( wheelRotations )
        meters = wheelRadians * SwerveModuleConstants.Drive.kWheelRadius
        return meters

    def __getDriveWheelVelocity(self, motorRotationsPerSecond:float) -> meters_per_second:
        wheelRotationsPerSecond = motorRotationsPerSecond * SwerveModuleConstants.Drive.kGearRatio
        wheelRadiansPerSec = rotationsToRadians( wheelRotationsPerSecond )
        metersPerSec = wheelRadiansPerSec * SwerveModuleConstants.Drive.kWheelRadius
        return metersPerSec
    def __calcDesiredMotorVelocity(self, wheelMetersPerSecond:float) -> float:
        """:returns: rotations per second of the motor"""
        motorMetersPerSecond = wheelMetersPerSecond / SwerveModuleConstants.Drive.kGearRatio
        motorRadiansPerSecond = motorMetersPerSecond / SwerveModuleConstants.Drive.kWheelRadius
        motorRotationsPerSecond = radiansToRotations( motorRadiansPerSecond )
        return motorRotationsPerSecond

    def __getTurnEncoderRotation(self) -> Rotation2d:
        return Rotation2d.fromRotations( self.__turnEncoder.get_absolute_position().value_as_double )

    def __getTurnEncoderRadians(self) -> radians:
        return rotationsToRadians( self.__turnEncoder.get_absolute_position().value_as_double )
