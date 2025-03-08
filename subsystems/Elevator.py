from commands2 import Subsystem
from wpimath.units import *
from wpimath.system.plant import DCMotor
from wpilib.shuffleboard import Shuffleboard
from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController, Color
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from enum import Enum
from util import FalconLogger
from rev import SparkMax, SparkBase, SparkMaxConfig, ClosedLoopConfig, ClosedLoopSlot, SparkMaxSim, LimitSwitchConfig

class ElevatorPositions(Enum):
    TROUGH = 0.0
    LOW_CORAL = 0.0
    MED_CORAL = 0.0
    HIGH_CORAL = 0.0

class ElevatorConstants:
    _kP = 1.0
    _kI = 0.0
    _kD = 0.0
    _kG = 0.0 # force to overcome gravity
    _kS = 0.0 # force to overcome friction
    _kV = 0.0 # Apply __ voltage for target velocity
    _kFF = 0.0 # Feed Forward

    _kOffset = 0.0
    _kTolerance = 0.0

    _elevatorHeight = 3
    _elevatorWidth = 3

    _gearing = 0.25
    _mass = 10.0
    _drumRadius = 0.25
    _rotsPerInch = 1.0
    _minLength = 0
    _maxLength = 0.5

class Elevator(Subsystem):
    def __init__(self):

        self.__motorLead = SparkMax(0, SparkBase.MotorType.kBrushless)
        self.__motorFollow = SparkMax(16, SparkBase.MotorType.kBrushless)

        self.__pidController = self.__motorLead.getClosedLoopController()

        self.__encoderLead = self.__motorLead.getEncoder()
        self.__encoderFollow = self.__motorFollow.getEncoder()

        self.__limitTop = self.__motorLead.getReverseLimitSwitch()
        self.__limitBottom = self.__motorLead.getForwardLimitSwitch()


        self.__motorLeadConfig = SparkMaxConfig()
        self.__motorLeadConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1)
        self.__motorLeadConfig.closedLoop.setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).P(ElevatorConstants._kP, ClosedLoopSlot.kSlot0).I(
            ElevatorConstants._kI, ClosedLoopSlot.kSlot0).D(
            ElevatorConstants._kD, ClosedLoopSlot.kSlot0).velocityFF(
            ElevatorConstants._kFF, ClosedLoopSlot.kSlot0).outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        
        self.__motorLeadConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        self.__motorLeadConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
        self.__motorLeadConfig.limitSwitch.forwardLimitSwitchEnabled(False)
        self.__motorLeadConfig.limitSwitch.reverseLimitSwitchEnabled(True)

        self.__motorLead.configure(self.__motorLeadConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
        self.__motorFollowConfig = SparkMaxConfig().follow(0, False)
        self.__motorFollow.configure(self.__motorFollowConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)

        self.__gearbox = DCMotor.NEO(2)
        self.__motorSim = SparkMaxSim(self.__motorLead, self.__gearbox)
        self.__elevatorSim = ElevatorSim(
            self.__gearbox,
            ElevatorConstants._gearing,
            ElevatorConstants._mass,
            ElevatorConstants._drumRadius,
            ElevatorConstants._minLength,
            ElevatorConstants._maxLength,
            True,
            0,
            [0.01, 0.00]
        )

        self.mech = Mechanism2d(
            30,
            50,
            Color8Bit(Color.kBlueViolet)
        )

        self.mechRoot = self.mech.getRoot(
            "ElevatorRoot",
            10,
            10
        )

        self.__setpoint = 0.0

        self.mechBase = self.mechRoot.appendLigament("ElevatorBase", 10, 0, color=Color8Bit(Color.kRed))
        self.mechElevator = self.mechBase.appendLigament("ElevatorImmutable", 10, 90, color=Color8Bit(Color.kRed))
        self.mechElevatorMutable = self.mechElevator.appendLigament("ElevatorMutable", 0, 0)

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

        FalconLogger.logOutput("Elevator/CurrentHeightIN", self.getPosition())
        FalconLogger.logOutput("Elevator/TargetHeightIN", self.getSetpoint())
    
    #TODO: Check if correct, feel like something is missing here
    def simulationPeriodic(self):
        #TODO: check if deadband needed
        self.__motorSim.setBusVoltage(RobotController.getBatteryVoltage())
        self.__elevatorSim.setInput([self.__motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()])
        self.__elevatorSim.update(0.020)
        self.__motorSim.iterate(
            radiansPerSecondToRotationsPerMinute(self.__elevatorSim.getVelocity()),
            RoboRioSim.getVInVoltage(),
            0.020
        )
        RoboRioSim.setVInVoltage(
            BatterySim.calculate([self.__elevatorSim.getCurrentDraw()])
        )
        self.mechElevatorMutable.setLength(
            self.__elevatorSim.getPosition()
        )

        
    def getSetpoint(self) -> float:
        return self.__setpoint
    
    def setSetpoint(self, sp: float) -> None:
        self.__setpoint = sp
        self.__pidController.setReference(
            sp,
            SparkBase.ControlType.kPosition, 
            ClosedLoopSlot.kSlot0
        )

    def atSetpoint(self) -> bool:
        margin = self.__encoderLead.getPosition()
        return abs(margin) <= ElevatorConstants._kTolerance
    
    def stop(self) -> None:
        self.setSetpoint(self.getPosition())

    def getPosition(self) -> float:
        """
        Get the current position of the elevator
        """
        return self.__encoderLead.getPosition()

    
    # per 0.020 seconds
    def calculateSimMotorRots(self) -> float:
        # for 3V, 100 rps
        mv = self.__motorSim.getAppliedOutput()
        return (mv/3) * 1.5
