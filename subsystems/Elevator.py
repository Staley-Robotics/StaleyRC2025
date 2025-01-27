from commands2 import Subsystem
from wpimath.units import *
from wpimath.system.plant import DCMotor
from wpilib.shuffleboard import Shuffleboard
from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController, Color
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from enum import Enum
from util import FalconLogger
from rev import SparkMax, SparkBase, SparkMaxConfig, ClosedLoopConfig, ClosedLoopSlot, SparkMaxSim

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
    _kV = 0.0 # Apply __ voltage for target velocitr
    _kFF = 0.0 # Feed Forward

    _kOffset = 0.0
    _kTolerance = 0.0

    _elevatorHeight = 3
    _elevatorWidth = 3

    _gearing = 1.0
    _mass = 10.0
    _drumRadius = 0.25
    _rotsPerInch = 1.0
    _minLength = 0
    _maxLength = 0.5

class Elevator(Subsystem):
    def __init__(self):

        self.__motorLead = SparkMax(0, SparkBase.MotorType.kBrushless)
        self.__motorFollow = SparkMax(1, SparkBase.MotorType.kBrushless)

        self.__pidController = self.__motorLead.getClosedLoopController()

        self.__encoderLead = self.__motorLead.getEncoder()

        self.__motorLeadConfig = SparkMaxConfig()
        self.__motorLeadConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1)
        self.__motorLeadConfig.closedLoop.setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        ).P(ElevatorConstants._kP, ClosedLoopSlot.kSlot0).I(
            ElevatorConstants._kI, ClosedLoopSlot.kSlot0).D(
            ElevatorConstants._kD, ClosedLoopSlot.kSlot0).velocityFF(
            ElevatorConstants._kFF, ClosedLoopSlot.kSlot0).outputRange(-1, 1, ClosedLoopSlot.kSlot0)

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
        FalconLogger.logInput("Elevator/MotorInput", self.__motorLead.get()) # current speed of motor
        FalconLogger.logInput("Elevator/MotorOutput", self.__motorLead.getAppliedOutput()) # TODO: check if this is the voltage
        FalconLogger.logInput("Elevator/MotorPositionRots", self.__encoderLead.getPosition())
        FalconLogger.logInput("Elevator/MotorVelocity", self.__encoderLead.getVelocity())

        FalconLogger.logOutput("Elevator/CurrentHeightIN", self.get())
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
        self.setSetpoint(self.get())

    def get(self) -> float:
        return self.__encoderLead.getPosition()

    # TODO: reimplement these so that calculations are mainly for setting, not writing for logging
    def inchesToElevatorPosition(self, pos: float) -> float:
        return (pos * 4) - 8

    def elevatorPositionToInches(self, pos: float) -> float:
        return (pos + 8) / 4
    
    # per 0.020 seconds
    def calculateSimMotorRots(self) -> float:
        # for 3V, 100 rps
        mv = self.__motorSim.getAppliedOutput()
        return (mv/3) * 1.5