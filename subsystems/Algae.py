import math

from commands2 import PIDSubsystem

from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController
from wpilib.shuffleboard import Shuffleboard
from wpimath.controller import PIDController, SimpleMotorFeedforwardRadians
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, degrees, degreesToRotations, rotationsToDegrees

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, \
    SensorDirectionValue

from rev import SparkMax, SparkBase, SparkMaxConfig

from enum import Enum

from util import FalconLogger

class AlgaeManipulatorPositions:
    MAX = 55.041
    HOLD = 30.041
    PLACE = 0.041
    GRAB = -20.041
    MIN = -52.031

class IntakeState(Enum):
    IN = 0.5
    OUT = -0.5
    OFF = 0

class AlgaeManipulatorConstants:
    # Pivot Constants
    pivot_kP: float = 25.0
    pivot_kI: float = 0.0
    pivot_kD: float = 0.0
    pivot_kFF: float = 0.0
    pivot_kV: float = 0.0
    pivot_kTolerance: float = 0.01

    pivot_kGearRatio: float = 1.0
    pivot_kOffsetRotations: float = 0.0


    # Intake Constants
    intake_kP: float = 25.0
    intake_kI: float = 0.0
    intake_kD: float = 0.0
    intake_kS: float = 0.1
    intake_kV: float = 0.0
    intake_kTolerance: float = 0.01

    intake_kGearRatio: float = 1.0
    intake_kOffsetRotations: float = 0.0
    class KrakenSim:
        kMaxRps = radiansToRotations(DCMotor.krakenX60(1).freeSpeed)


class AlgaeManipulator(PIDSubsystem):
    # Motors (Neo and 775pro, at 25:1 and 10.3333:1 respectively)
    __pivotMotor:SparkMax = None
    pivotSetPoint = 0.0

    __intakeMotor: TalonFX = None
    intakeState = IntakeState.OFF

    def __init__(self):
        # Motor
        # Neo
        self.__pivotController = self.__pivotMotor.getClosedLoopController()
        self.sparkConfig = SparkMaxConfig()
        self.sparkConfig.closedLoop.pidf(AlgaeManipulatorConstants.pivot_kP, AlgaeManipulatorConstants.pivot_kI, AlgaeManipulatorConstants.pivot_kD, AlgaeManipulatorConstants.pivot_kFF)


        # 775pro
        self.__intakeMotor = TalonFX(25, "canivore1")

        # Zero Motor
        self.voltOut = VoltageOut(0, use_timesync=True)
        self.dutyOut = DutyCycleOut(0, use_timesync=True)

        # PID Controller
        pidController = PIDController(AlgaeManipulatorConstants.pivot_kP, AlgaeManipulatorConstants.pivot_kI, AlgaeManipulatorConstants.pivot_kD)
        # pidController.setTolerance(AlgaeManipulatorConstants.kTolerance)
        # pidController.enableContinuousInput(-0.5, 0.5)

        super().__init__(
            pidController,
        )

        # Enable Subsystem PIDController
        self.enable()

        # Mechanism Graphics / Logging NOT IMPLEMENTED YET
        # self.mech = Mechanism2d(28, 28, Color8Bit(0, 0, 0))
        #
        #
        #

        Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulator", self)
        # Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulatorMech", self.mech)
        Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulatorPid", self._controller)

    def periodic(self) -> None:
        # Input Logging
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorInput", self.__pivotMotor.get())
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorOutput", self.__pivotMotor.getAppliedOutput())
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorPosition_abs", self.__pivotMotor.getAbsoluteEncoder().getPosition())
        FalconLogger.logInput("AlgaeManipulator/Pivot/ClosedLoopController", self.__pivotMotor.getClosedLoopController())

        # Input Logging
        FalconLogger.logInput("AlgaeManipulator/Intake/MotorInput", self.__intakeMotor.get())
        FalconLogger.logInput("AlgaeManipulator/Intake/MotorOutput", self.__intakeMotor.get_motor_voltage())

        # Run
        if RobotState.isDisabled():
            self.stop()
        super().periodic()

        # Visualization | MORE UNIMPLEMENTED MECHANISM 2D
        # offset = self.mechPost.getAngle()
        # self.mechFront.setAngle(rotationsToDegrees(self.getMeasurement()) - offset)
        # self.mechBack.setAngle(rotationsToDegrees(self.getMeasurement()) + 180 - offset)

        # Output Logging
        FalconLogger.logOutput("AlgaeManipulator/Pivot/TargetAngle", rotationsToDegrees(self.getSetpoint()))
        FalconLogger.logOutput("AlgaeManipulator/Pivot/ActualAngle", rotationsToDegrees(self.getMeasurement()))

        FalconLogger.logOutput("AlgaeManipulator/Intake/State", self.intakeActive)
        FalconLogger.logOutput("AlgaeManipulator/Intake/Voltage", self.__intakeMotor.get_motor_voltage())

    # def simulationPeriodic(self) -> None:
    #     # Simulation Motor Deadband
    #     if self.atSetpoint() and abs(self.__motor.get_duty_cycle().value) <= 0.011:
    #         self.__motor.set_control(self.dutyOut.with_output(0.0))
    #
    #     # Motor
    #     self.__motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
    #     velocity = self.__motor.sim_state.motor_voltage / 12 * AlgaeManipulatorConstants.KrakenSim.kMaxRps
    #
    #     self.__motor.sim_state.set_rotor_velocity(velocity)
    #     self.__motor.sim_state.add_rotor_position(velocity * 0.02)
    #
    #     # CANcoder
    #     self.__encoder.sim_state.set_velocity(-velocity * AlgaeManipulatorConstants.kGearRatio)
    #     self.__encoder.sim_state.add_position(-velocity * AlgaeManipulatorConstants.kGearRatio * 0.02)

    def stop(self) -> None:
        self.setSetpoint(rotationsToDegrees(self.getMeasurement()), True)
        self.__intakeMotor.set(0.0)

    def setSetpoint(self, setpoint: degrees, overrideRange: bool = False):
        setpoint *= AlgaeManipulatorConstants.pivot_kGearRatio # Gear Ratio
        # Limits the specific range (Protects mechanism)
        if not overrideRange:
            setpoint = min(max(setpoint, AlgaeManipulatorPositions.MIN), AlgaeManipulatorPositions.MAX)
        setpoint = degreesToRotations(setpoint)
        return super().setSetpoint(setpoint)

    def getMeasurement(self) -> float:
        return self.__pivotMotor.getAbsoluteEncoder().getPosition()

    def atSetpoint(self, position: float = None) -> bool:
        if position is None or self.getSetpoint() == degreesToRotations(position):
            return self._controller.atSetpoint()
        else:
            return False

    def setIntake(self, state: IntakeState):
        self.intakeState = state
        self.__intakeMotor.set(state.value)