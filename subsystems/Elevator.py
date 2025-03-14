from commands2 import Subsystem
from wpimath.units import *
from wpimath.system.plant import DCMotor
from wpilib.shuffleboard import Shuffleboard
from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController, Color
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from enum import Enum
from util import FalconLogger
from rev import SparkMax, SparkBase, SparkMaxConfig, ClosedLoopConfig, ClosedLoopSlot, SparkMaxSim, LimitSwitchConfig, EncoderConfig

class ElevatorPositions: #(Enum):
    MIN:inches = 33.0
    COLLECT:inches = 37.0
    L1:inches = 34.0
    L2:inches = 45.0
    L3:inches = 60.0
    L4:inches = 70.0
    MAX:inches = 72.00

class ElevatorConstants:
    _kP = 3.0
    _kI = 0.0
    _kD = 0.1
    _kG = 0.0 # force to overcome gravity
    _kS = 0.0 # force to overcome friction
    _kV = 0.0 # Apply __ voltage for target velocity
    _kFF = 0.0 # Feed Forward

    _kTolerance = 0.250

    _gearing:float = 4 #1 / 4
    _mass:kilograms = 10.0
    _drumRadius:inches = 2.256 / 2 # Belt Gear Diameter / 2
    _minHeight:inches = 33.00
    _maxHeight:inches = 72.00

class Elevator(Subsystem):
    __useNativeMotorUnits:bool = False
    __setpoint:float = 0.0

    def __init__(self):
        posConvFactor:inches = 1 if self.__useNativeMotorUnits else ( 1 / ElevatorConstants._gearing ) * ( 2 * ElevatorConstants._drumRadius * math.pi )
        velConvFactor:float = 1 if self.__useNativeMotorUnits else ( posConvFactor / 60 )

        self.__motorLead = SparkMax(0, SparkBase.MotorType.kBrushless)
        self.__motorFollow = SparkMax(16, SparkBase.MotorType.kBrushless)

        self.__pidController = self.__motorLead.getClosedLoopController()

        self.__encoderLead = self.__motorLead.getEncoder()
        self.__encoderFollow = self.__motorFollow.getEncoder()

        self.__limitTop = self.__motorLead.getReverseLimitSwitch()
        self.__limitBottom = self.__motorLead.getForwardLimitSwitch()

        motorLcfg = SparkMaxConfig()
        motorFcfg = SparkMaxConfig()

        motorLcfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        
        motorFcfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        motorFcfg.follow( self.__motorLead.getDeviceId(), False )

        en_cfg = EncoderConfig()
        en_cfg = en_cfg.positionConversionFactor( posConvFactor ).velocityConversionFactor( velConvFactor )
        
        cl_cfg = ClosedLoopConfig()
        cl_cfg = cl_cfg.setFeedbackSensor( ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder )
        cl_cfg = cl_cfg.pidf(
            ElevatorConstants._kP,
            ElevatorConstants._kI,
            ElevatorConstants._kD,
            ElevatorConstants._kFF,
            ClosedLoopSlot.kSlot0
        )
        cl_cfg = cl_cfg.outputRange( -1.0, 1.0, ClosedLoopSlot.kSlot0 )
        
        ls_cfg = LimitSwitchConfig()
        ls_cfg = ls_cfg.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        ls_cfg = ls_cfg.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
        ls_cfg = ls_cfg.forwardLimitSwitchEnabled(False)
        ls_cfg = ls_cfg.reverseLimitSwitchEnabled(False)
                
        motorLcfg.apply( cl_cfg )
        motorLcfg.apply( ls_cfg )
        motorLcfg.apply( en_cfg )

        motorFcfg.apply( cl_cfg )
        motorFcfg.apply( ls_cfg )
        motorFcfg.apply( en_cfg )

        self.__motorLead.configure(motorLcfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
        self.__motorFollow.configure(motorFcfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)

        encoderSetpoint = 0 if self.__useNativeMotorUnits else ElevatorConstants._minHeight
        self.__encoderLead.setPosition( encoderSetpoint )
        self.__encoderFollow.setPosition( encoderSetpoint )

        self.stop()
        #self.setSetpoint( ElevatorConstants._minHeight )           
        
        # Simulation
        self.__motorSimLead = SparkMaxSim(self.__motorLead, DCMotor.NEO(1))
        self.__motorSimLead.setPosition( encoderSetpoint )
        self.__motorSimLead.enable()
        
        self.__motorSimLeadEncoder = self.__motorSimLead.getRelativeEncoderSim()
        self.__motorSimLeadEncoder.setPosition( encoderSetpoint )

        self.__motorSimFollow = SparkMaxSim(self.__motorFollow, DCMotor.NEO(1))
        self.__motorSimFollow.setPosition( encoderSetpoint )
        self.__motorSimFollow.enable()

        self.__motorSimFollowEncoder = self.__motorSimFollow.getRelativeEncoderSim()
        self.__motorSimFollowEncoder.setPosition( encoderSetpoint )
        
        self.__elevatorSim = ElevatorSim(
            DCMotor.NEO(2),
            ElevatorConstants._gearing,
            ElevatorConstants._mass,
            inchesToMeters( ElevatorConstants._drumRadius ),
            inchesToMeters( ElevatorConstants._minHeight ),
            inchesToMeters( ElevatorConstants._maxHeight ),
            True,
            inchesToMeters( ElevatorConstants._minHeight ),
            #[0.01, 0.00]
        )
        self.__elevatorSim.setState( inchesToMeters( ElevatorConstants._minHeight ), 0.0 )

        # Mechanism
        self.mech = Mechanism2d(
            30,
            100,
            Color8Bit(Color.kWhite)
        )

        self.mechRoot = self.mech.getRoot(
            "ElevatorTarget",
            20,
            0
        )

        self.mechBase = self.mechRoot.appendLigament("ElevatorBase", ElevatorConstants._minHeight, 90, lineWidth = 10, color=Color8Bit(Color.kRed))
        self.mechElevatorTarget = self.mechBase.appendLigament("Elevator_(1)_Target", self.__setpoint - ElevatorConstants._minHeight, 0, lineWidth = 10, color = Color8Bit(Color.kYellow))
        self.mechElevatorActual = self.mechBase.appendLigament("Elevator_(2)_Actual", self.getPosition() - ElevatorConstants._minHeight, 0, lineWidth = 5, color = Color8Bit(Color.kGreen))

        SmartDashboard.putData("Elevator", self)
        SmartDashboard.putData("ElevatorMech", self.mech)


    def periodic(self):
        # Lead motor
        FalconLogger.logInput("Elevator/Lead/MotorInput", self.__motorLead.get()) # current speed of motor
        FalconLogger.logInput("Elevator/Lead/MotorOutput", self.__motorLead.getAppliedOutput())
        FalconLogger.logInput("Elevator/Lead/MotorPosition_r", self.__encoderLead.getPosition())
        FalconLogger.logInput("Elevator/Lead/MotorVelocity_rpm", self.__encoderLead.getVelocity())
        FalconLogger.logInput("Elevator/Lead/MotorCurrent_a", self.__motorLead.getOutputCurrent())
        FalconLogger.logInput("Elevator/Lead/MotorTemp_c", self.__motorLead.getMotorTemperature())

        # Follow motor
        FalconLogger.logInput("Elevator/Follow/MotorInput", self.__motorFollow.get())  # current speed of motor
        FalconLogger.logInput("Elevator/Follow/MotorOutput", self.__motorFollow.getAppliedOutput())
        FalconLogger.logInput("Elevator/Follow/MotorPosition_r", self.__encoderFollow.getPosition())
        FalconLogger.logInput("Elevator/Follow/MotorVelocity_rpm", self.__encoderFollow.getVelocity())
        FalconLogger.logInput("Elevator/Follow/MotorCurrent_a", self.__motorFollow.getOutputCurrent())
        FalconLogger.logInput("Elevator/Follow/MotorTemp_c", self.__motorFollow.getMotorTemperature())

        # Limit Switches
        FalconLogger.logInput("Elevator/TopLimitSwitch", self.__limitTop.get())
        FalconLogger.logInput("Elevator/BottomLimitSwitch", self.__limitBottom.get())

        # Using Hardware based PID so no run function is needed. May need to look into a stop function though for disabled
        self.mechElevatorActual.setLength( self.getPosition() - ElevatorConstants._minHeight )
        self.mechElevatorTarget.setLength( self.getSetpoint() - ElevatorConstants._minHeight )

        FalconLogger.logOutput("Elevator/ActualHeight_i", self.getPosition())
        FalconLogger.logOutput("Elevator/TargetHeight_i", self.getSetpoint())
    

    def simulationPeriodic(self):
        velocity_mps = self.__elevatorSim.getVelocity()
        velocity_ips = metersToInches( velocity_mps )
        velocity_radps_drum = velocity_ips / ElevatorConstants._drumRadius
        velocity_rotps_drum = radiansToRotations( velocity_radps_drum )
        velocity_rotps_motor = velocity_rotps_drum * ElevatorConstants._gearing
        velocity_rotpm_motor = velocity_rotps_motor * 60

        FalconLogger.logOutput("Elevator/SimHeight_i", self.__elevatorSim.getPositionInches() )
        FalconLogger.logOutput("Elevator/SimVelocity_linear_ips", velocity_ips )
        FalconLogger.logOutput("Elevator/SimVelocity_motor_rpm", velocity_rotpm_motor )

        # Lead Motor
        self.__motorSimLead.iterate(
            velocity_rotpm_motor if self.__useNativeMotorUnits else velocity_ips,
            self.__motorSimLead.getBusVoltage(),
            0.020
        )

        # Follower Motor
        self.__motorSimFollow.iterate(
            velocity_rotpm_motor if self.__useNativeMotorUnits else velocity_ips,
            self.__motorSimLead.getBusVoltage(),
            0.020
        )

        appOut = self.__motorSimLead.getAppliedOutput()
        # if not RobotState.isEnabled(): appOut = 0.0
        # elif abs( appOut ) < 0.01: appOut = 0.0

        self.__elevatorSim.setInputVoltage( appOut * 12.0 )
        self.__elevatorSim.update(0.020)

        
    def getSetpoint(self) -> inches:
        return self.__setpoint
    
    def setSetpoint(self, sp: inches, override: bool = False ) -> None:
        self.__setpoint = min( max( sp, ElevatorConstants._minHeight ), ElevatorConstants._maxHeight )
        if override: self.__setpoint = sp

        if self.__useNativeMotorUnits: # use rotations
            height = ( self.__setpoint - ElevatorConstants._minHeight )
            heightDrum_rad = height / ElevatorConstants._drumRadius
            heightMotor_rad = heightDrum_rad * ElevatorConstants._gearing
            heightMotor_rot = radiansToRotations( heightMotor_rad )

            self.__pidController.setReference(
                heightMotor_rot, # rotations
                SparkBase.ControlType.kPosition, 
                ClosedLoopSlot.kSlot0
            )
        else: # use inches
            self.__pidController.setReference(
                self.__setpoint, # inches
                SparkBase.ControlType.kPosition, 
                ClosedLoopSlot.kSlot0
            )

    def getPosition(self) -> inches:
        """
        Get the current position of the elevator
        """
        if self.__useNativeMotorUnits: # rotations to inches
            motor_rot = self.__encoderLead.getPosition()
            drum_rot = motor_rot / ElevatorConstants._gearing
            drum_rad = rotationsToRadians( drum_rot )
            offsetHeight_i = drum_rad * ElevatorConstants._drumRadius
            calcHeight_i = offsetHeight_i + ElevatorConstants._minHeight
            return calcHeight_i
        else: # already in inches
            return self.__encoderLead.getPosition()       

    def atSetpoint(self) -> bool:
        sp = self.getSetpoint()
        cur = self.getPosition()
        return abs(cur - sp) <= ElevatorConstants._kTolerance
    
    def stop(self) -> None:
        self.setSetpoint( self.getPosition() )
    
    def reset(self) -> None:
        self.__encoderLead.setPosition( ElevatorPositions.MIN )
        self.__encoderFollow.setPosition( ElevatorPositions.MIN )
    
    # per 0.020 seconds
    # def calculateSimMotorRots(self) -> float:
    #     # for 3V, 100 rps
    #     mv = self.__motorSim.getAppliedOutput()
    #     return (mv/3) * 1.5
