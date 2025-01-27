from commands2 import Subsystem
from wpimath.units import *
from wpimath.system.plant import DCMotor
from wpilib.shuffleboard import Shuffleboard
from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController, Color
from wpilib.simulation import ElevatorSim, RoboRioSim, BatterySim
from enum import Enum
from util import FalconLogger
from rev import SparkMax, SparkBase, SparkMaxConfig, ClosedLoopConfig, ClosedLoopSlot

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

class Elevator(Subsystem):
    def __init__(self):

        self.__motorLead = SparkMax(0, SparkBase.MotorType.kBrushless)
        self.__motorFollow = SparkMax(1, SparkBase.MotorType.kBrushless)

        self.__encoderLead = self.__motorLead.getEncoder()
        self.__encoderFollower = self.__motorFollow.getEncoder()

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

        self.pid_controller = self.__motorLead.getClosedLoopController()

        self.__motorFollow.PersistMode

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

        self.mechBase = self.mechRoot.appendLigament("ElevatorBase", 10, 0, color=Color8Bit(Color.kRed))
        self.mechElevator = self.mechBase.appendLigament("ElevatorImmutable", 10, 90, color=Color8Bit(Color.kRed))
        self.mechElevatorMutable = self.mechElevator.appendLigament("ElevatorMutable", 0, 0)

        SmartDashboard.putData("Elevator", self)
        SmartDashboard.putData("ElevatorMech", self.mech)


    def periodic(self):
        FalconLogger.logInput("Elevator/MotorInput", self.__m2.get())
        FalconLogger.logInput("Elevator/MotorOutput", self.__m2.get_motor_voltage().value)
        FalconLogger.logInput("Elevator/MotorPositionRots", self.__m2.get_position().value_as_double)
        FalconLogger.logInput("Elevator/MotorVelocityRPS", self.__m2.get_velocity().value)

        FalconLogger.logOutput("Elevator/CurrentHeightIN", self.get())
        FalconLogger.logOutput("Elevator/TargetHeightIN", self.getSetpoint())
    
    #TODO: Check if correct, feel like something is missing here
    def simulationPeriodic(self):
        #TODO: check if deadband needed
        self.__m2.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
        self.__m2.sim_state.add_rotor_position(self.calculateSimMotorRots())
        self.mechElevatorMutable.setLength(
            self.get()
        )

        
    def getSetpoint(self) -> float:
        return self.elevatorPositionToInches(self.__m2.get_closed_loop_reference().value_as_double)
        # return self.elevatorPositionToInches(self.__setpoint)
    
    def setSetpoint(self, sp: float) -> None:
        req = PositionVoltage(self.inchesToElevatorPosition(sp))
        self.__m2.set_control(req)

    def atSetpoint(self) -> bool:
        margin = self.__m2.get_closed_loop_error().value_as_double
        self.__m2.get_closed_loop_reference
        return abs(margin) <= ElevatorConstants._kTolerance
    
    def stop(self) -> None:
        self.setSetpoint(self.get())

    def get(self) -> float:
        return self.elevatorPositionToInches(self.__m2.get_position().value_as_double)

    # TODO: reimplement these so that calculations are mainly for setting, not writing for logging
    def inchesToElevatorPosition(self, pos: float) -> float:
        return (pos * 4) - 8

    def elevatorPositionToInches(self, pos: float) -> float:
        return (pos + 8) / 4
    
    # per 0.020 seconds
    def calculateSimMotorRots(self) -> float:
        # for 3V, 100 rps
        mv = self.__m2.sim_state.motor_voltage
        return (mv/3) * 1.5

    def updateMech(self) -> None:
        self.mechElevatorMutable.setLength(self.get())