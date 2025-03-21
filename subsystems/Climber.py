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
    NotInUse:degrees = 200.0
    Prepare:degrees = 5.0
    Climbing:degrees= 180.0

class ClimberConstants:
    kP = 0#5.0
    kI = 0.0
    kD = 0.0
    kFF = 0#0.5 # Feed Forward
    kArbFF = 0.0
    kTolerance:degrees = 1.0

    kP_Climb = 0#5.0
    kI_Climb = 0.0
    kD_Climb = 0.0
    kFF_Climb = 0.0 # Feed Forward
    kArbFF_Climb = 0.0   

    gearRatio:float = 75.6
    armLength_m:meters = 0.2667
    armMass_kg:kilograms = 1.35575924
    minAngle_d:degrees = -75.0
    maxAngle_d:degrees = 200.0
    useGravity:bool = True
    startAngle_d:float = 90.0

class Climber(Subsystem):
    
    setpoint:float = 0.0
    control_type:SparkMax.ControlType = None

    # Initialization
    def __init__(self, mainId:int, followId:int, encoder_offset:float) -> None:
        '''
        mainId: main motor ID\n
        followId: following motor ID
        '''
        ## Init Motors
        self.__leadMotor:SparkMax = SparkMax( mainId, SparkMax.MotorType.kBrushless )
        self.__followMotor:SparkMax = SparkMax( followId, SparkMax.MotorType.kBrushless )

        self.__encoder:SparkAbsoluteEncoder = self.__leadMotor.getAbsoluteEncoder()
        self.__controller:SparkClosedLoopController = self.__leadMotor.getClosedLoopController()
        
        # Init motors and config
        lMotorCfg = SparkMaxConfig()
        lMotorCfg = lMotorCfg.setIdleMode( SparkMaxConfig.IdleMode.kCoast )

        fMotorCfg = SparkMaxConfig()
        fMotorCfg = fMotorCfg.setIdleMode( SparkMaxConfig.IdleMode.kCoast )
        fMotorCfg = fMotorCfg.follow( self.__leadMotor.getDeviceId() )
        
        clConfig = ClosedLoopConfig()
        clConfig = clConfig.pidf(
            ClimberConstants.kP,
            ClimberConstants.kI,
            ClimberConstants.kD,
            ClimberConstants.kFF,
            ClosedLoopSlot.kSlot0
        )
        clConfig = clConfig.pidf(
            ClimberConstants.kP_Climb,
            ClimberConstants.kI_Climb,
            ClimberConstants.kD_Climb,
            ClimberConstants.kFF_Climb,
            ClosedLoopSlot.kSlot1
        )
        clConfig = clConfig.setFeedbackSensor( ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder )
        clConfig = clConfig.positionWrappingEnabled(False).positionWrappingInputRange(0, 1)
        
        encConfig = AbsoluteEncoderConfig()
        encConfig = encConfig.inverted( True )
        encConfig = encConfig.zeroOffset( encoder_offset )
        encConfig = encConfig.zeroCentered( True )

        # Apply configs
        lMotorCfg.apply(clConfig)
        lMotorCfg.apply(encConfig)
        
        self.__leadMotor.configure( lMotorCfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters )
        self.__followMotor.configure( fMotorCfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters )
        
        self.control_type = SparkMax.ControlType.kPosition

        # Mechanism2d
        mech = Mechanism2d( 30, 40, Color8Bit(50,50,70) )
        mechRoot = mech.getRoot("Climber", 15, 0)
        mechBase = mechRoot.appendLigament("ClimberPost", 6, 90, 2, color=Color8Bit(Color.kGray))
        self.mechClimberTarget = mechBase.appendLigament("ClimberActual", 8, 90, 2, color=Color8Bit(Color.kYellow))
        self.mechClimberActual = mechBase.appendLigament("ClimberTarget", 4, 90, 3, color=Color8Bit(Color.kGreen))
        if RobotBase.isSimulation(): self.mechClimberSim = mechBase.appendLigament("ClimberSSim", 6, 90, 3, color=Color8Bit(Color.kRed))

        # Shuffleboard
        SmartDashboard.putData( "Climber", self )
        SmartDashboard.putData( "ClimberMech", mech )

        # Simulation
        self.simLMotor = SparkMaxSim(self.__leadMotor, DCMotor.NEO(1))
        # self.simLMotor.setPosition( degreesToRotations( 90 ) )

        self.simClimber = SingleJointedArmSim(
            DCMotor.NEO(2),
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
        FalconLogger.logInput("Climber/LeadMotorInput", self.__leadMotor.get())
        FalconLogger.logInput("Climber/LeadMotorOutput", self.__leadMotor.getAppliedOutput())
        FalconLogger.logInput("Climber/LeadMotorCurrent_a", self.__leadMotor.getOutputCurrent())
        FalconLogger.logInput("Climber/LeadMotorPosition_abs_r", self.__leadMotor.getEncoder().getPosition())
        FalconLogger.logInput("Climber/LeadMotorVelocity_rpm", self.__leadMotor.getEncoder().getPosition())
        FalconLogger.logInput("Climber/LeadMotorTemp_c", self.__leadMotor.getMotorTemperature())
        
        FalconLogger.logInput("Climber/FollowMotorInput", self.__leadMotor.get())
        FalconLogger.logInput("Climber/FollowMotorOutput", self.__leadMotor.getAppliedOutput())
        FalconLogger.logInput("Climber/FollowMotorCurrent_a", self.__leadMotor.getOutputCurrent())
        FalconLogger.logInput("Climber/FollowMotorPosition_abs_r", self.__leadMotor.getEncoder().getPosition())
        FalconLogger.logInput("Climber/FollowMotorVelocity_rpm", self.__leadMotor.getEncoder().getPosition())
        FalconLogger.logInput("Climber/FollowMotorTemp_c", self.__leadMotor.getMotorTemperature())

        FalconLogger.logInput("Climber/EncoderPosition_r", self.__encoder.getPosition())
        FalconLogger.logInput("Climber/EncoderVelocity_rpm", self.__encoder.getVelocity()) # TODO should this be rps? or maybe do a * 60

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()

        self.mechClimberActual.setAngle( self.getPosition() - 90.0 )
        self.mechClimberTarget.setAngle( self.getSetpoint() - 90.0 )

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
        if self.control_type == SparkMax.ControlType.kPosition:
            sp = degreesToRotations( self.getSetpoint() )

            arbFF = ClimberConstants.kArbFF_Climb if self.isClimbing() else ClimberConstants.kArbFF
            cosineScalar = math.cos( degreesToRadians( self.getPosition() ) )
            slot = ClosedLoopSlot.kSlot1 if self.isClimbing() else ClosedLoopSlot.kSlot0       

            self.__controller.setReference(
                sp,
                SparkMax.ControlType.kPosition,
                slot,
                arbFF * cosineScalar,
                SparkClosedLoopController.ArbFFUnits.kVoltage
            )
        else: # kDutyCycle
            # Safeties - check if position nearing bounds, only allow to move away from bound
            pos = self.getPosition() + 90.0
            if pos > 360.0: pos -= 360.0
            if pos < 0.0: pos += 360.0

            if pos <= 90.0: #>= ClimberPositions.Climbing - ClimberConstants.kTolerance:
                self.setSetpoint(max(0, self.getSetpoint()), True)
            elif pos >= 270.0: #ClimberPositions.Prepare - ClimberConstants.kTolerance: # tolerance subtracted to extend farther if needed
                self.setSetpoint(min(0, self.getSetpoint()), True)


            # print(self.getSetpoint(), self.control_type)
            # assumes setpoint will be overrided to match duty cycle range
            self.__controller.setReference(
                self.getSetpoint(),
                self.control_type,
            )

    # Stop the Subsystem
    def stop(self) -> None:
        self.setSetpoint( self.getPosition(), True )

    # Set the Desired Position
    def setSetpoint(self, position:degrees, override:bool = False):
        """Sets the desired position of the climber
    
        :param position: the desired position in degrees
        :param override: whether or not this uses min(max()) to mnormalize position
        """
        if not override:
            position = min(max(position, ClimberConstants.minAngle_d), ClimberConstants.maxAngle_d )
        self.setpoint = position

    # Get Desired Position
    def getSetpoint(self):
        """Returns desired position of the climber in degrees (not climber's motor, the actual climber)"""
        return self.setpoint
    
    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        """Returns true if climber is at desired position (not climber's motor, the actual climber)"""
        return abs(self.getPosition() - self.getSetpoint()) < ClimberConstants.kTolerance
    
    def getPosition(self) -> float:
        """
        Gets position of clmiber in degrees (not climber's motor, the actual climber)

        :returns: position of climber in degrees
        """
        val = rotationsToDegrees(self.__encoder.getPosition())

        # if val > 270:
        #     val -= 360
        # elif val < -90:
        #     val += 360
            
        return val
        
    def isClimbing(self) -> bool:
        return False