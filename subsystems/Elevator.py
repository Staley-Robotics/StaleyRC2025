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
    _kP = 2.0
    _kI = 0.0
    _kD = 0.1
    _kG = 0.0 # force to overcome gravity
    _kS = 0.0 # force to overcome friction
    _kV = 0.0 # Apply __ voltage for target velocity
    _kFF = 0.0 # Feed Forward

    _kOffset = 0.0
    _kTolerance = 0.125

    _beltGearDiameter_in = 2.256

    _gearing = 4 #1 / 4
    _mass = 10.0
    _drumRadius = _beltGearDiameter_in / 2 # 0.25
    _rotsPerInch = 1 / ( _beltGearDiameter_in * math.pi )
    _minLength = 33.00
    _maxLength = 72.00

class Elevator(Subsystem):
    def __init__(self):

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
        motorFcfg.follow( self.__motorLead.getDeviceId() , False )

        en_cfg = EncoderConfig()
        en_cfg.positionConversionFactor(1).velocityConversionFactor(1)
        
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
        ls_cfg.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        ls_cfg.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
        ls_cfg.forwardLimitSwitchEnabled(False)
        ls_cfg.reverseLimitSwitchEnabled(True)
                
        motorLcfg.apply( cl_cfg )
        motorLcfg.apply( ls_cfg )
        motorLcfg.apply( en_cfg )

        motorFcfg.apply( cl_cfg )
        motorFcfg.apply( ls_cfg )
        motorFcfg.apply( en_cfg )

        self.__motorLead.configure(motorLcfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
        self.__motorFollow.configure(motorFcfg, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
        
        # Simulation
        self.__motorSimLead = SparkMaxSim(self.__motorLead, DCMotor.NEO(1))
        self.__motorSimLeadEncoder = self.__motorSimLead.getRelativeEncoderSim()
        self.__motorSimFollow = SparkMaxSim(self.__motorFollow, DCMotor.NEO(1))
        self.__motorSimFollowEncoder = self.__motorSimFollow.getRelativeEncoderSim()
        self.__elevatorSim = ElevatorSim(
            DCMotor.NEO(2),
            ElevatorConstants._gearing,
            ElevatorConstants._mass,
            inchesToMeters( ElevatorConstants._drumRadius ),
            inchesToMeters( ElevatorConstants._minLength ),
            inchesToMeters( ElevatorConstants._maxLength ),
            False,
            inchesToMeters( ElevatorConstants._minLength ),
            #[0.01, 0.00]
        )
        self.__motorSimLead.enable()
        self.__motorSimFollow.enable()

        # Mechanism
        self.mech = Mechanism2d(
            30,
            100,
            Color8Bit(Color.kBlueViolet)
        )

        self.mechRoot = self.mech.getRoot(
            "ElevatorTarget",
            20,
            0
        )

        self.__setpoint = ElevatorConstants._minLength

        self.mechBase = self.mechRoot.appendLigament("ElevatorBase", ElevatorConstants._minLength, 90, lineWidth = 10, color=Color8Bit(Color.kRed))
        self.mechElevatorTarget = self.mechBase.appendLigament("ElevatorTarget", self.__setpoint - ElevatorConstants._minLength, 5, lineWidth = 10, color = Color8Bit(Color.kYellow))
        self.mechElevatorActual = self.mechBase.appendLigament("ElevatorActual", self.getPosition() - ElevatorConstants._minLength, 355, lineWidth = 6, color = Color8Bit(Color.kGreen))

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

        self.setSetpoint( self.getSetpoint() )
        # Using Hardware based PID so no run function is needed. May need to look into a stop function though for disabled
        self.mechElevatorTarget.setLength( self.getSetpoint() - ElevatorConstants._minLength )
        self.mechElevatorActual.setLength( self.getPosition() - ElevatorConstants._minLength )

        FalconLogger.logOutput("Elevator/CurrentHeightIN", self.getPosition())
        FalconLogger.logOutput("Elevator/TargetHeightIN", self.getSetpoint())
    
    #TODO: Check if correct, feel like something is missing here
    def simulationPeriodic(self):
        #TODO: check if deadband needed
        self.__elevatorSim.setInputVoltage( self.__motorSimLead.getAppliedOutput() * 12.0 )
        self.__elevatorSim.update(0.020)

        velocity_mps = self.__elevatorSim.getVelocity()
        velocity_ips = metersToInches( velocity_mps )
        velocity_rotps = velocity_ips * ElevatorConstants._rotsPerInch
        velocity_rotps_motor = velocity_rotps * ElevatorConstants._gearing
        velocity_rotpm_motor = velocity_rotps_motor * 60
        velocity_radps = velocity_ips / ( ElevatorConstants._beltGearDiameter_in / 2.0 )
        velocity_RPM = radiansPerSecondToRotationsPerMinute( velocity_radps ) * ElevatorConstants._gearing

        FalconLogger.logOutput("Elevator/Sim_Velocity_ips", metersToInches( self.__elevatorSim.getVelocity() ) )
        FalconLogger.logOutput("Elevator/Sim_Velocity_RPM_motor", velocity_RPM )
        FalconLogger.logOutput("Elevator/Sim_Height_i", self.__elevatorSim.getPositionInches() )

        # Lead Motor
        self.__motorSimLead.iterate(
            velocity_rotpm_motor, #radiansPerSecondToRotationsPerMinute(self.__elevatorSim.getVelocity()),
            RoboRioSim.getVInVoltage(),
            0.020
        )
        # self.__motorSimLeadEncoder.iterate(
        #     velocity_rotpm_motor, #radiansPerSecondToRotationsPerMinute(self.__elevatorSim.getVelocity()),
        #     0.020
        # )

        # # Follower Motor
        self.__motorSimFollow.iterate(
            velocity_rotps_motor, #radiansPerSecondToRotationsPerMinute(self.__elevatorSim.getVelocity()),
            RoboRioSim.getVInVoltage(),
            0.020
        )
        # self.__motorSimFollowEncoder.iterate(
        #     velocity_rotpm_motor, #radiansPerSecondToRotationsPerMinute(self.__elevatorSim.getVelocity()),
        #     0.020
        # )

        
    def getSetpoint(self) -> inches:
        return self.__setpoint
    
    def setSetpoint(self, sp: inches, override: bool = False ) -> None:
        if override: self.__setpoint
        else: self.__setpoint = min( max( sp, ElevatorConstants._minLength ), ElevatorConstants._maxLength )

        motorPosition = ( sp - ElevatorConstants._minLength ) * ElevatorConstants._rotsPerInch * ElevatorConstants._gearing

        self.__pidController.setReference(
            motorPosition,
            SparkBase.ControlType.kPosition, 
            ClosedLoopSlot.kSlot0
        )

    def getPosition(self) -> inches:
        """
        Get the current position of the elevator
        """
        rot = self.__encoderLead.getPosition()
        rotDrum = rot / ElevatorConstants._gearing
        motInches = rotDrum / ElevatorConstants._rotsPerInch
        phyInches = motInches + ElevatorConstants._minLength
        return phyInches #self.__encoderLead.getPosition()

    def atSetpoint(self) -> bool:
        sp = self.getSetpoint()
        cur = self.getPosition()
        return abs(cur - sp) <= ElevatorConstants._kTolerance
    
    def stop(self) -> None:
        self.setSetpoint( self.getPosition() )
    
    # per 0.020 seconds
    # def calculateSimMotorRots(self) -> float:
    #     # for 3V, 100 rps
    #     mv = self.__motorSim.getAppliedOutput()
    #     return (mv/3) * 1.5
