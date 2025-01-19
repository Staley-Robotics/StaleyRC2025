from commands2 import Subsystem
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, Slot0Configs
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, SensorDirectionValue
from phoenix6.controls import Follower, PositionVoltage
from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.units import *
from wpilib import RobotBase
from enum import Enum
from util import FalconLogger

def inchesToElevatorPosition(pos: float) -> float:
    return pos

def elevatorPositionToInches(pos: float) -> float:
    return pos

class ElevatorPositions(Enum):
    TROUGH = 0.0
    LOW_CORAL = 0.0
    MED_CORAL = 0.0
    HIGH_CORAL = 0.0

class ElevatorConstants:
    __kP = 0.0
    __kI = 0.0
    __kD = 0.0
    __kG = 0.0 # force to overcome gravity
    __kS = 0.0 # force to overcome friction
    __kV = 0.0 # Apply __ voltage for target velocitr

    __kOffset = 0.0
    __kTolerance = 0.0


class Elevator(Subsystem):
    __m1: TalonFX = None
    __m2: TalonFX = None
    __e1: CANcoder = None
    __e2: CANcoder = None

    def __init__(self):
        sl0Cfg = Slot0Configs()
        sl0Cfg.k_p = ElevatorConstants.__kP
        sl0Cfg.k_i = ElevatorConstants.__kI
        sl0Cfg.k_d = ElevatorConstants.__kD
        sl0Cfg.k_g = ElevatorConstants.__kG

        self.__m1 = Follower(0, True)

        stdMotorCfg = TalonFXConfiguration()
        stdMotorCfg.motor_output.neutral_mode = NeutralModeValue.BRAKE
        stdMotorCfg.motor_output.duty_cycle_neutral_deadband = 0.001
        stdMotorCfg.with_slot0(sl0Cfg)

        self.__m2 = TalonFX(0, "idk")
        self.__m2.configurator.apply(stdMotorCfg)

        # initialization assumes elevator is at the bottom

        self.__m2.get_position()
        self.__req = PositionVoltage(0).with_slot(0)
        self.__setpoint = 0.0

        super().__init__()

    def periodic(self):
        # TODO: Log what is occuring in motors
        FalconLogger.logInput("Elevator/MotorInput", self.__m2.get())
        FalconLogger.logInput("Elevator/MotorOutput", self.__m2.get_motor_voltage().value)
        FalconLogger.logInput("Elevator/MotorPositionRots", self.__m2.get_position().value)
        FalconLogger.logInput("Elevator/MotorVelocityRPS", self.__m2.get_velocity().value)
        self.__m2.set_control(self.__req.with_position(inchesToElevatorPosition(self.__setpoint)))
    
    def simulationPeriodic(self):
        return super().simulationPeriodic()

    def getSetpoint(self) -> float:
        return elevatorPositionToInches(self.__setpoint)
    
    def setSetpoint(self, sp: float) -> None:
        self.__setpoint = sp

    def atSetpoint(self) -> bool:
        margin = self.__m2.get_closed_loop_error()
        return abs(margin) <= ElevatorConstants.__kTolerance
    
    def stop(self) -> None:
        pass

    def get(self) -> float:
        return elevatorPositionToInches(self.__m2.get_position().value_as_double)
    
    def set(self, pos: float) -> None:
        pos = inchesToElevatorPosition(pos)
        
        self.__m2.set_control(self.__req.with_position(pos))


