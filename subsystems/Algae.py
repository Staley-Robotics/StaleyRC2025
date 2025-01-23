import math

from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController
from wpilib.shuffleboard import Shuffleboard
from wpimath.controller import PIDController, SimpleMotorFeedforwardRadians
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, degrees, degreesToRotations, rotationsToDegrees, kSecondsPerMinute

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, \
    SensorDirectionValue

from rev import SparkMax, SparkMaxConfig, SparkMaxSim, SparkBase

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
    class NeoSim:
        kMaxRpm = DCMotor.NEO(1).freeSpeed * kSecondsPerMinute

    class Vex775Sim:
        kMaxRpm = DCMotor.vex775Pro().freeSpeed * kSecondsPerMinute


class AlgaeManipulator():
    # Motors (Neo and 775pro, at 25:1 and 10.3333:1 respectively)
    __pivotMotor:SparkMax = None
    pivotSetpoint:float = None

    __intakeMotor: TalonFX = None
    intakeState = IntakeState.OFF

    def __init__(self):
        # Motor
        # Neo
        self.__pivotMotor = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.__pivotController = self.__pivotMotor.getClosedLoopController()
        self.sparkConfig = SparkMaxConfig()
        self.sparkConfig.closedLoop.pidf(AlgaeManipulatorConstants.pivot_kP, AlgaeManipulatorConstants.pivot_kI, AlgaeManipulatorConstants.pivot_kD, AlgaeManipulatorConstants.pivot_kFF)


        # 775pro
        self.__intakeMotor = TalonFX(25, "canivore1")

        # Zero Motor
        self.voltOut = VoltageOut(0, use_timesync=True)
        self.dutyOut = DutyCycleOut(0, use_timesync=True)

        if RobotBase.isSimulation():
            # Pivot Simulation
            self.simPivot = SparkMaxSim(self.__pivotMotor, DCMotor.NEO(1))
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

        Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulator", self)
        # Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulatorMech", self.mech)

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

        # Intake
        self.__intakeMotor.set(self.intakeState.value)

        # Output Logging
        FalconLogger.logOutput("AlgaeManipulator/Pivot/TargetAngle", rotationsToDegrees(self.getSetpoint()))
        FalconLogger.logOutput("AlgaeManipulator/Pivot/ActualAngle", rotationsToDegrees(self.getMeasurement()))

        FalconLogger.logOutput("AlgaeManipulator/Intake/State", self.intakeState)
        FalconLogger.logOutput("AlgaeManipulator/Intake/Voltage", self.__intakeMotor.get_motor_voltage())

    def simulationPeriodic(self) -> None:
        # Neo Periodic
        driveRpm = AlgaeManipulatorConstants.NeoSim.kMaxRpm * self.__pivotMotor.get()
        self.simPivot.getRelativeEncoderSim().iterate(driveRpm, 0.02)
        self.simPivot.getRelativeEncoderSim().setVelocity(driveRpm)
        self.simPivot.getAbsoluteEncoderSim().setVelocity(driveRpm)
        self.simPivot.getAbsoluteEncoderSim().setPosition(self.simPivot.getRelativeEncoderSim().getPosition() % 1)

        # 775pro Periodic
        velocity = AlgaeManipulatorConstants.Vex775Sim.kMaxRpm * self.__intakeMotor.get()
        self.simIntake.sim_state.set_rotor_velocity(velocity)
        self.simIntake.sim_state.add_rotor_position(velocity * 0.02)


    def stop(self) -> None:
        self.setSetpoint(rotationsToDegrees(self.getMeasurement()), True)
        self.__intakeMotor.set(0.0)

    def setSetpoint(self, setpoint: degrees, overrideRange: bool = False):
        setpoint *= AlgaeManipulatorConstants.pivot_kGearRatio # Gear Ratio
        # Limits the specific range (Protects mechanism)
        if not overrideRange:
            setpoint = min(max(setpoint, AlgaeManipulatorPositions.MIN), AlgaeManipulatorPositions.MAX)
        self.__pivotController.setReference(degreesToRotations(setpoint), SparkBase.ControlType.kPosition)
        self.pivotSetpoint = setpoint


    def getMeasurement(self) -> float:
        return self.__pivotMotor.getAbsoluteEncoder().getPosition()

    def atSetpoint(self, positionDeg: float = None) -> bool:
        if positionDeg is None:
            positionDeg = rotationsToDegrees(self.getMeasurement())
        return self.pivotSetpoint == positionDeg

    def getSetpoint(self) -> float:
        return self.pivotSetpoint

    def setIntake(self, state: IntakeState):
        self.intakeState = state
        self.__intakeMotor.set(state.value)