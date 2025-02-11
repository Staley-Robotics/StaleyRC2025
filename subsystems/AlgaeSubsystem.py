import math

from commands2 import Subsystem
from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController, Victor, DigitalInput
from wpilib.shuffleboard import Shuffleboard
from wpimath.controller import PIDController, SimpleMotorFeedforwardRadians
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, degrees, degreesToRotations, rotationsToDegrees, kSecondsPerMinute

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, \
    SensorDirectionValue

from phoenix5 import TalonSRX, TalonSRXControlMode

from rev import SparkMax, SparkMaxConfig, SparkMaxSim, SparkBase, ClosedLoopConfig

from enum import Enum

from util import FalconLogger


class AlgaeManipulatorPositions:
    MAX = 95.0
    HOLD = 90.0
    PLACE = 95.0
    GRAB = 35.0
    MIN = 0.0


class IntakeState(Enum):
    IN = 0.5
    OUT = -0.5
    OFF = 0


class AlgaeManipulatorConstants:
    # Pivot Constants
    pivot_kP: float = .01
    pivot_kI: float = 0.0
    pivot_kD: float = 0.0
    pivot_kFF: float = 0.0
    pivot_kV: float = 0.0
    pivot_kTolerance: float = 0.01

    pivot_kGearRatio: float = 25
    pivot_kOffsetRotations: float = 0.0

    # Intake Constants
    intake_kP: float = 1.0
    intake_kI: float = 0.0
    intake_kD: float = 0.0
    intake_kS: float = 0.1
    intake_kV: float = 0.0
    intake_kTolerance: float = 0.01

    intake_kGearRatio: float = 10.3333
    intake_kOffsetRotations: float = 0.0

    class NeoSim:
        kMaxRpm = DCMotor.NEO(2).freeSpeed * kSecondsPerMinute

    class Vex775Sim:
        kMaxRpm = DCMotor.vex775Pro().freeSpeed * kSecondsPerMinute


class AlgaeManipulator(Subsystem):
    # Motors (Neo and 775pro, at 25:1 and 10.3333:1 respectively)
    __pivotMotorOne: SparkMax = None
    __pivotMotorTwo: SparkMax = None
    pivotSetpoint: float = None

    __intakeMotor: TalonSRX = None
    intakeState = IntakeState.OFF

    __irBeam:DigitalInput = None

    def __init__(self):
        # Motor
        # Neo
        self.__pivotMotorOne = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.__pivotMotorTwo = SparkMax(2, SparkMax.MotorType.kBrushless)
        self.__pivotController = self.__pivotMotorOne.getClosedLoopController()
        self.sparkConfig = SparkMaxConfig()
        self.sparkConfig.closedLoop.pidf(AlgaeManipulatorConstants.pivot_kP, AlgaeManipulatorConstants.pivot_kI,
                                         AlgaeManipulatorConstants.pivot_kD, AlgaeManipulatorConstants.pivot_kFF)
        self.sparkConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.__pivotMotorOne.configure(self.sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                                       SparkBase.PersistMode.kNoPersistParameters)
        self.sparkConfig.follow(1, True)
        self.__pivotMotorTwo.configure(self.sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                                       SparkBase.PersistMode.kNoPersistParameters)

        # 775pro
        self.__intakeMotor = TalonSRX(25)
        # Zero Motor
        self.voltOut = VoltageOut(0, use_timesync=True)
        self.dutyOut = DutyCycleOut(0, use_timesync=True)

        self.mech = Mechanism2d(3, 3, Color8Bit(0, 0, 0))
        self.root = self.mech.getRoot("Pivot", 2, 0.5)
        self.manipulator = self.root.appendLigament("Manipulator", 1, 0, 0.5, Color8Bit(255, 255, 255))

        SmartDashboard.putData("AlgaeMech", self.mech)

        if RobotBase.isSimulation():
            # Pivot Simulation
            self.simPivot = SparkMaxSim(self.__pivotMotorOne, DCMotor.NEO(1))
            self.simPivot.getAbsoluteEncoderSim().setPositionConversionFactor(1)
            self.simPivot.getAbsoluteEncoderSim().setVelocityConversionFactor(1)
            self.simPivot.getRelativeEncoderSim().setPositionConversionFactor(1)
            self.simPivot.getRelativeEncoderSim().setVelocityConversionFactor(1)

            # Intake Simulation
            self.simIntake = TalonFX(25, "canivore1")

        # Mechanism Graphics / Logging NOT IMPLEMENTED YET
        # self.mech = Mechanism2d(28, 28, Color8Bit(0, 0, 0))
        #
        #
        #

        # IR Beam
        self.__irBeam = DigitalInput(3)

        # Shuffleboard
        Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulator", self)
        # Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulatorMech", self.mech)

    def periodic(self) -> None:
        # Input Logging
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorInput", self.__pivotMotorOne.get())
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorOutput", self.__pivotMotorOne.getAppliedOutput())
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorPosition_abs",
                              self.__pivotMotorOne.getAbsoluteEncoder().getPosition())

        # Input Logging
        FalconLogger.logInput("AlgaeManipulator/Intake/MotorOutputPercent", self.__intakeMotor.getMotorOutputPercent())
        FalconLogger.logInput("AlgaeManipulator/Intake/MotorVoltage", self.__intakeMotor.getMotorOutputVoltage())

        # Input Logging
        FalconLogger.logInput("AlgaeManipulator/IRBeam", self.__irBeam.get())

        # Run
        if RobotState.isDisabled():
            self.stop()

        # Intake
        self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, self.intakeState.value)

        # Output Logging
        FalconLogger.logOutput("AlgaeManipulator/Pivot/TargetAngle", self.getSetpoint())
        FalconLogger.logOutput("AlgaeManipulator/Pivot/ActualAngle", self.getMeasurement())

        FalconLogger.logOutput("AlgaeManipulator/Intake/DesiredState", self.intakeState.value)
        FalconLogger.logOutput("AlgaeManipulator/Intake/CurrentState", self.__intakeMotor.getMotorOutputPercent())
        FalconLogger.logOutput("AlgaeManipulator/Intake/Voltage", self.__intakeMotor.getMotorOutputVoltage())

    def simulationPeriodic(self) -> None:
        # Neo Periodic
        driveRpm = AlgaeManipulatorConstants.NeoSim.kMaxRpm * self.__pivotMotorOne.getAppliedOutput()
        self.simPivot.iterate(driveRpm, 12, 0.02)
        self.simPivot.getRelativeEncoderSim().iterate(driveRpm, 0.02)
        self.simPivot.getAbsoluteEncoderSim().iterate(driveRpm / AlgaeManipulatorConstants.pivot_kGearRatio, 0.02)

        self.manipulator.setAngle(self.getMeasurement())
        # self.simPivot.getRelativeEncoderSim().setVelocity(driveRpm)
        # self.simPivot.getAbsoluteEncoderSim().setVelocity(driveRpm)
        # self.simPivot.getAbsoluteEncoderSim().setPosition(self.simPivot.getRelativeEncoderSim().getPosition() % 1)

        # 775pro Periodic
        velocity = AlgaeManipulatorConstants.Vex775Sim.kMaxRpm * self.__intakeMotor.getMotorOutputPercent()
        self.simIntake.sim_state.set_rotor_velocity(velocity)
        self.simIntake.sim_state.add_rotor_position(velocity * 0.02)

    def stop(self) -> None:
        self.setSetpoint(self.getMeasurement(), True)
        self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)

    def setSetpoint(self, setpoint: degrees, overrideRange: bool = False):
        # Limits the specific range (Protects mechanism)
        if not overrideRange:
            setpoint = min(max(setpoint, AlgaeManipulatorPositions.MIN), AlgaeManipulatorPositions.MAX)
        self.pivotSetpoint = setpoint
        self.__pivotController.setReference(degreesToRotations(setpoint), SparkBase.ControlType.kPosition)

    def getMeasurement(self) -> float:
        return rotationsToDegrees(self.__pivotMotorOne.getAbsoluteEncoder().getPosition())

    def atSetpoint(self, positionDeg: float = None) -> bool:
        if positionDeg is None:
            positionDeg = self.getMeasurement()
        return self.pivotSetpoint == positionDeg

    def getSetpoint(self) -> float:
        return self.pivotSetpoint

    def setIntake(self, state: IntakeState):
        self.intakeState = state
        self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, state.value)

    def hasAlgae(self) -> bool:
        return not self.__irBeam.get()