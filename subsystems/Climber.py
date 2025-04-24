from math import pi
from enum import Enum

from wpilib import RobotState, Mechanism2d, SmartDashboard, Color, Color8Bit, XboxController, RobotBase
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from wpimath.units import *

from commands2 import Subsystem

from rev import SparkBase, SparkMax, SparkMaxSim, SparkMaxConfig, AbsoluteEncoderConfig, ClosedLoopSlot, ClosedLoopConfig, SparkAbsoluteEncoder, SparkClosedLoopController

from util import FalconLogger

class ClimberSpeeds:
    STOP:float = 0
    FORWARD:float = 1
    BACKWARD:float = -1

class ClimberPositions:
    MaxPosition:degrees = 200 # TODO: measure
    MinPosition:degrees =  -20

class ClimberConstants:

    safeTolerance:degrees = 10.0

    gearRatio:float = 100
    armLength_m:meters = 0.2667
    armMass_kg:kilograms = 1.35575924
    minAngle_d:degrees = -60.0
    maxAngle_d:degrees = 200.0
    useGravity:bool = True
    startAngle_d:float = 90.0

class Climber(Subsystem):
    
    # setpoint = 0.0
    control_type:SparkMax.ControlType = None

    # Initialization
    def __init__(self, motorId:int, encoder_offset:float) -> None:
        '''
        mainId: motor ID\n
        '''
        ## Init Motor
        self.__motor:SparkMax = SparkMax( motorId, SparkMax.MotorType.kBrushless )

        self.__setSpeed:percent = 0

        self.__encoder:SparkAbsoluteEncoder = self.__motor.getAbsoluteEncoder()
        # self.__controller:SparkClosedLoopController = self.__motor.getClosedLoopController()
        
        # Init motors and config
        lMotorCfg = SparkMaxConfig()
        lMotorCfg = lMotorCfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        
        encConfig = AbsoluteEncoderConfig()
        encConfig = encConfig.inverted( True )
        encConfig = encConfig.positionConversionFactor(ClimberConstants.gearRatio).velocityConversionFactor(ClimberConstants.gearRatio / 60)
        encConfig = encConfig.zeroOffset( encoder_offset )
        encConfig = encConfig.zeroCentered( True )

        # Apply configs
        self.__motor.configure( lMotorCfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters )
        
        # Mechanism2d
        mech = Mechanism2d( 30, 40, Color8Bit(50,50,70) )
        mechRoot = mech.getRoot("Climber", 15, 0)
        mechBase = mechRoot.appendLigament("ClimberPost", 6, 90, 2, color=Color8Bit(Color.kGray))
        # self.mechClimberTarget = mechBase.appendLigament("ClimberTarget", 8, 90, 2, color=Color8Bit(Color.kYellow))
        self.mechClimberActual = mechBase.appendLigament("ClimberActual", 4, 90, 3, color=Color8Bit(Color.kGreen))
        if RobotBase.isSimulation(): self.mechClimberSim = mechBase.appendLigament("ClimberSSim", 6, 90, 3, color=Color8Bit(Color.kRed))

        # Shuffleboard
        SmartDashboard.putData( "Climber", self )
        SmartDashboard.putData( "ClimberMech", mech )

        # Simulation
        self.simLMotor = SparkMaxSim(self.__motor, DCMotor.NEO(1))
        # self.simLMotor.setPosition( degreesToRotations( 90 ) )

        self.simClimber = SingleJointedArmSim(
            DCMotor.NEO(1),
            ClimberConstants.gearRatio,
            SingleJointedArmSim.estimateMOI( ClimberConstants.armLength_m, ClimberConstants.armMass_kg ),
            ClimberConstants.armLength_m,
            degreesToRadians( ClimberConstants.minAngle_d ),
            degreesToRadians( ClimberConstants.maxAngle_d ),
            ClimberConstants.useGravity,
            degreesToRadians( ClimberConstants.startAngle_d )
        )
        self.simClimber.setState( degreesToRadians( ClimberConstants.startAngle_d ), 0.0 )

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Log Inputs
        FalconLogger.logInput("Climber/LeadMotorInput", self.__motor.get())
        FalconLogger.logInput("Climber/LeadMotorOutput", self.__motor.getAppliedOutput())
        FalconLogger.logInput("Climber/LeadMotorCurrent_a", self.__motor.getOutputCurrent())
        FalconLogger.logInput("Climber/LeadMotorPosition_abs_r", self.__motor.getEncoder().getPosition())
        FalconLogger.logInput("Climber/LeadMotorVelocity_rpm", self.__motor.getEncoder().getPosition())
        FalconLogger.logInput("Climber/LeadMotorTemp_c", self.__motor.getMotorTemperature())
        
        FalconLogger.logInput("Climber/FollowMotorInput", self.__motor.get())
        FalconLogger.logInput("Climber/FollowMotorOutput", self.__motor.getAppliedOutput())
        FalconLogger.logInput("Climber/FollowMotorCurrent_a", self.__motor.getOutputCurrent())
        FalconLogger.logInput("Climber/FollowMotorPosition_abs_r", self.__motor.getEncoder().getPosition())
        FalconLogger.logInput("Climber/FollowMotorVelocity_rpm", self.__motor.getEncoder().getPosition())
        FalconLogger.logInput("Climber/FollowMotorTemp_c", self.__motor.getMotorTemperature())

        FalconLogger.logInput("Climber/EncoderPosition_r", self.__encoder.getPosition())
        FalconLogger.logInput("Climber/EncoderVelocity_rpm", self.__encoder.getVelocity()) # TODO should this be rps? or maybe do a * 60

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()

        self.mechClimberActual.setAngle( self.getPosition() - 90.0 )
        # self.mechClimberTarget.setAngle( self.getSetpoint() - 90.0 )

        # Logging: Log Outputs
        FalconLogger.logOutput("Climber/TargetPosition_d", self.getSetpoint())
        FalconLogger.logOutput("Climber/ActualPosition_d", self.getPosition()) 

    # Simulation Periodic Loop
    def simulationPeriodic(self):

        """
        Get voltage from motors (in volts)
        Apply output to sim class (sim class will be a single jointed arm)
        Apply sim output to motor sim object

        """

        velocity_radps = self.simClimber.getVelocity() # note: this returns velocity in radians per sec
        velocity_rpm = radiansToRotations( velocity_radps ) * 60

        self.simLMotor.setMotorCurrent(0)
        self.simLMotor.iterate( velocity_rpm, 12.0, 0.02 )
        self.simLMotor.getRelativeEncoderSim().iterate( velocity_rpm * ClimberConstants.gearRatio, 0.02)

        self.mechClimberSim.setAngle(self.simClimber.getAngleDegrees() - 90)

        FalconLogger.logInput("Climber/SimVelocity_rpm", velocity_rpm)
        FalconLogger.logInput("Climber/SimPosition_r", radiansToRotations( self.simClimber.getAngle() ))

        ## Update MOI while you are Climbing
        # if self.isClimbing():
        #     self.simPivotArm.setInput()
        # else:
        #     self.simPivotArm.estimateMOI()

        self.simClimber.setInputVoltage( self.simLMotor.getAppliedOutput() * 12 )
        self.simClimber.update(0.02)

    # Run the Subsystem
    def run(self) -> None:
        # Safeties - check if position nearing bounds, only allow to move away from bound
        if self.getPosition() >= ClimberPositions.MaxPosition:
            self.setSetpoint(min(0, self.getSetpoint()))
        elif self.getPosition() <= ClimberPositions.MinPosition:
            self.setSetpoint(max(0, self.getSetpoint()))

        self.__motor.set(self.__setSpeed)

    # Stop the Subsystem
    def stop(self) -> None:
        self.setSetpoint( self.getPosition(), True )

    # Set the Desired Position
    def setSetpoint(self, speed:float, ovveride:bool=False):
        self.__setSpeed = speed

    # Get Desired Position
    def getSetpoint(self):
        """Returns desired speed percentage of the motor"""
        return self.__setSpeed
    
    # Check if Subsystem is at the Desired State
    # def atSetpoint(self) -> bool:
    #     """Returns true if climber is at desired position (not climber's motor, the actual climber)"""
    #     return abs(self.getPosition() - self.getSetpoint()) < ClimberConstants.kTolerance
    
    def getPosition(self) -> float:
        """
        Gets position of clmiber in degrees (not climber's motor, the actual climber)

        :returns: position of climber in degrees
        """
        val = rotationsToDegrees(self.__encoder.getPosition())
            
        return val
        
    # def isClimbing(self) -> bool:
    #     return False