import typing
import math

from wpilib import RobotBase
from wpilib.shuffleboard import Shuffleboard
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, SimpleMotorFeedforwardMeters, SimpleMotorFeedforwardRadians
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath import applyDeadband
from wpimath.system.plant import DCMotor
from wpimath.units import radiansPerSecondToRotationsPerMinute, rotationsToRadians, radiansToRotations, kSecondsPerMinute, radians, meters_per_second, meters

from rev import SparkMax, SparkRelativeEncoder, SparkMaxSim, SparkMaxConfig

from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration
from phoenix6.controls import VelocityVoltage, PositionVoltage
from phoenix6.signals.spn_enums import SensorDirectionValue, NeutralModeValue, InvertedValue
#hey im writing things
from util import FalconLogger

class SwerveModuleConstants:
    class Drive:
        kWheelRadius:float = 0.0508
        kGearRatio:float = 1 / 6.75
        kP:float = 1 #0.1
        kI:float = 0
        kD:float = 0
        kS:float = 0 #0.1
        kV:float = 2.4#2.8235 #0.13
    
    class Turn:
        kGearRatio:float = 1 / (150/7)
        kP:float = 4.2#2.5 # 25.0
        kI:float = 0
        kD:float = 0
        kMaxAngularVelocity:float = math.pi
        kMaxAngularAcceleration:float = math.pi
        kS:float = 0
        kV:float = 0
    
    # class NeoSim:
    #     kMaxRpm:float = radiansToRotations( DCMotor.NEO(1).freeSpeed ) * kSecondsPerMinute
    class KrakenSim:
        kMaxRps:float = radiansToRotations( DCMotor.krakenX60(1).freeSpeed )# * kSecondsPerMinute

class SwerveModule:
    # Variable Declaration
    __driveMotor:TalonFX = None
    # __driveEncoder:typing.Any = None
    __drivePid:PIDController = None
    __driveFF:SimpleMotorFeedforwardMeters = None

    __turnMotor:TalonFX = None
    __turnEncoder:CANcoder = None
    __turnPid:ProfiledPIDControllerRadians = None
    __turnFF:SimpleMotorFeedforwardRadians = None

    __setpoint:SwerveModuleState = None

    def __init__(self, moduleId:int, driveId:int, turnId:int, encoderId:int, encoderOffset:float):
        # Module Name
        self.moduleId = moduleId
        
        # Drive Motor
        # basic config
        driveMotorCfg = TalonFXConfiguration()
        driveMotorCfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        driveMotorCfg.current_limits.with_stator_current_limit( 40.0 ).with_stator_current_limit_enable( True )
        driveMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # pid config
        driveMotorCfg.slot0.k_p = SwerveModuleConstants.Drive.kP
        driveMotorCfg.slot0.k_i = SwerveModuleConstants.Drive.kI
        driveMotorCfg.slot0.k_d = SwerveModuleConstants.Drive.kD
        #motor init
        self.__driveMotor = TalonFX( driveId, "rio" )
        self.__driveMotor.configurator.apply( driveMotorCfg )

        self.__driveMotor.set_control( VelocityVoltage(
            0,

        ) )

        self.__drivePid = PIDController( SwerveModuleConstants.Drive.kP, SwerveModuleConstants.Drive.kI, SwerveModuleConstants.Drive.kD )
        self.__drivePid.setTolerance( 0.01 )
        self.__driveFF = SimpleMotorFeedforwardMeters( SwerveModuleConstants.Drive.kS, SwerveModuleConstants.Drive.kV )
        

        # Turn Motor
        turnMotorCfg = TalonFXConfiguration()
        turnMotorCfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        turnMotorCfg.current_limits.with_stator_current_limit( 20.0 ).with_stator_current_limit_enable( True )
        turnMotorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.__turnMotor = TalonFX( turnId, "rio" )
        
        # Turn Encoder (CANcoder)
        turnEncoderCfg = CANcoderConfiguration()
        turnEncoderCfg.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        turnEncoderCfg.magnet_sensor.sensor_direction = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        if not RobotBase.isSimulation(): turnEncoderCfg.magnet_sensor.magnet_offset = encoderOffset
        self.__turnEncoder = CANcoder( encoderId, "rio" )
        self.__turnEncoder.configurator.apply( turnEncoderCfg )
        self.__turnEncoder.set_position( self.__turnEncoder.get_absolute_position().value ) # Position Safeguard

        # Turn PID
        self.__turnPid = PIDController( SwerveModuleConstants.Turn.kP, SwerveModuleConstants.Turn.kI, SwerveModuleConstants.Turn.kD )
        self.__turnPid.enableContinuousInput( -math.pi, math.pi )
        self.__turnPid.setTolerance( 0.150 )
        self.__turnFF = SimpleMotorFeedforwardRadians( SwerveModuleConstants.Turn.kS, SwerveModuleConstants.Turn.kV )

        # Drive Motor Simulation Setup
        # self.__driveMotorSim = SparkMaxSim( self.__driveMotor, DCMotor.NEO(1) )
        # self.__driveMotorSim.getAbsoluteEncoderSim().setPositionConversionFactor(1)
        # self.__driveMotorSim.getAbsoluteEncoderSim().setVelocityConversionFactor(1)
        # self.__driveMotorSim.getRelativeEncoderSim().setPositionConversionFactor(1)
        # self.__driveMotorSim.getRelativeEncoderSim().setVelocityConversionFactor(1)
        
        # Turn Motor Simulation Setup
        # self.__turnMotorSim = SparkMaxSim( self.__turnMotor, DCMotor.NEO(1) )
        # self.__turnMotorSim.getAbsoluteEncoderSim().setPositionConversionFactor(1)
        # self.__turnMotorSim.getAbsoluteEncoderSim().setVelocityConversionFactor(1)
        # self.__turnMotorSim.getRelativeEncoderSim().setPositionConversionFactor(1)
        # self.__turnMotorSim.getRelativeEncoderSim().setVelocityConversionFactor(1)
        
        # CANcoder Simulation Setup
        self.__turnEncoderSim = self.__turnEncoder.sim_state

        # Default Desired State
        self.__setpoint = SwerveModuleState(0, Rotation2d(0.0))

        # Dashboards
        Shuffleboard.getTab( "SwerveDrive" ).add( f"{moduleId}-DrivePid", self.__drivePid )
        Shuffleboard.getTab( "SwerveDrive" ).add( f"{moduleId}-TurnPid", self.__turnPid )

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
        driveVelocity = self.__getDriveVelocity( self.__driveMotor.get_velocity().value )
        driveOutput = self.__drivePid.calculate( driveVelocity, self.__setpoint.speed )
        driveOutputFF = self.__driveFF.calculate( self.__setpoint.speed )
        driveOut = driveOutput + driveOutputFF
        if RobotBase.isSimulation() and abs( driveOut ) < 0.12: driveOut = 0.0
        self.__driveMotor.setVoltage( driveOut )

        # Set Turn Motor
        turnOutput = self.__turnPid.calculate( self.__getTurnEncoderRadians(), self.__setpoint.angle.radians() )
        turnOutputFF = 0#self.__turnFF.calculate( self.__setpoint.angle.radians() )
        turnOut = turnOutput + turnOutputFF
        if RobotBase.isSimulation() and abs( turnOut ) < 0.12: turnOut = 0.0
        self.__turnMotor.setVoltage( turnOut )

    def runSim(self) -> None:
        # Drive Motor Position and Velocity
        driveRps = SwerveModuleConstants.KrakenSim.kMaxRps * self.__driveMotor.get()
        self.__driveMotor.sim_state.set_rotor_velocity(driveRps)
        self.__driveMotor.sim_state.add_rotor_position( driveRps * 0.02 ) # idk what this does, stole from 24v2 intake
        # self.__driveMotorSim.getRelativeEncoderSim().iterate( driveRpm, 0.02 )
        # self.__driveMotorSim.getRelativeEncoderSim().setVelocity( driveRpm )
        # self.__driveMotorSim.getAbsoluteEncoderSim().setVelocity( driveRpm )
        # self.__driveMotorSim.getAbsoluteEncoderSim().setPosition( self.__driveMotorSim.getRelativeEncoderSim().getPosition() % 1 )
        
        # Turn Motor Position and Velocity
        turnRps = SwerveModuleConstants.KrakenSim.kMaxRps * self.__turnMotor.get()
        self.__driveMotor.sim_state.set_rotor_velocity(driveRps)
        self.__driveMotor.sim_state.add_rotor_position( driveRps * 0.02 )
        # self.__turnMotorSim.getRelativeEncoderSim().iterate( turnRpm, 0.02 )
        # self.__turnMotorSim.getRelativeEncoderSim().setVelocity( turnRpm )
        # self.__turnMotorSim.getAbsoluteEncoderSim().setVelocity( turnRpm )
        # self.__turnMotorSim.getAbsoluteEncoderSim().setPosition( self.__turnMotorSim.getRelativeEncoderSim().getPosition() % 1 )

        # CANcoder Velocity and Position
        canRps = turnRps * SwerveModuleConstants.Turn.kGearRatio# / kSecondsPerMinute
        self.__turnEncoderSim.set_velocity( canRps )
        self.__turnEncoderSim.add_position( canRps * 0.02 )

    def setState(self, desiredState:SwerveModuleState) -> None:
        # Optimize
        currentRotation = self.__getTurnEncoderRotation()
        self.__setpoint = desiredState
        self.__setpoint.optimize( currentRotation )
        self.__setpoint.cosineScale( currentRotation ) # Smooth Out Turning

    def getState(self) -> SwerveModuleState:
        driveVelocity = self.__getDriveVelocity( self.__driveMotor.get_velocity().value )
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

    def __getDriveVelocity(self, motorRotationsPerSecond:float) -> meters_per_second:
        wheelRotationsPerSecond = motorRotationsPerSecond * SwerveModuleConstants.Drive.kGearRatio
        wheelRadiansPerSec = rotationsToRadians( wheelRotationsPerSecond )
        metersPerSec = wheelRadiansPerSec * SwerveModuleConstants.Drive.kWheelRadius
        return metersPerSec

    def __getTurnEncoderRotation(self) -> Rotation2d:
        return Rotation2d.fromRotations( self.__turnEncoder.get_absolute_position().value_as_double )

    def __getTurnEncoderRadians(self) -> radians:
        return rotationsToRadians( self.__turnEncoder.get_absolute_position().value_as_double )
