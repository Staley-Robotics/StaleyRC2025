from commands2 import PIDSubsystem

from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController
from wpilib.shuffleboard import Shuffleboard

from wpimath.controller import PIDController, SimpleMotorFeedforwardRadians
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, degrees, degreesToRotations, rotationsToDegrees

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, SensorDirectionValue

from util import FalconLogger

class ElevatorPositions:
    MIN = 0
    L1 = 1
    L2 = 2
    L3 = 3
    L4 = 4
    MAX = 5

class ElevatorConstants:
    kP = 0.0
    kI = 0.0
    kD = 0.0
    kS = 0.0
    kV = 0.0
    kTolerance = 0.01

    kGearRatio = 1.0
    kOffsetRotations = 0.0

    class KrakenSim:
        kMaxRps = radiansToRotations( DCMotor.krakenX60(1).freeSpeed )

class Elevator(PIDSubsystem):
    # Motors
    __motor: TalonFX = None
    __encoder: CANcoder = None

    def __init__(self):
        # Motor Config
        motorCfg = TalonFXConfiguration()
        motorCfg.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        motorCfg.motor_output.neutral_mode = NeutralModeValue.COAST
        motorCfg.motor_output.duty_cycle_neutral_deadband = 0.001
        self.__motor = TalonFX(25, "canivore1")
        self.__motor.configurator.apply(motorCfg)
        self.voltOut = VoltageOut(0, use_timesync=True)
        self.dutyOut = DutyCycleOut(0, use_timesync=True)

        # Encoder Config
        encoderCfg = CANcoderConfiguration()
        encoderCfg.magnet_sensor.absolute_sensor_discontinuity_point = 0.5
        encoderCfg.magnet_sensor.sensor_direction = SensorDirectionValue.CLOCKWISE_POSITIVE
        if not RobotBase.isSimulation(): encoderCfg.magnet_sensor.magnet_offset = ElevatorConstants.kOffsetRotations
        self.__encoder = CANcoder(26, "canivore1")
        self.__encoder.configurator.apply(encoderCfg)
        self.__encoder.set_position(
            self.__encoder.get_absolute_position().value)  # Protects against accidental reboot / value changes

        # PID Controller
        pidController = PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        pidController.setTolerance(ElevatorConstants.kTolerance)
        pidController.enableContinuousInput(-0.5, 0.5)

        super().__init__(
            pidController,
            self.__encoder.get_absolute_position().value
        )

        self.enable()

        # MECHANISM 2D would normally go here, but we're not using it yet
        #
        #

        # Shuffleboard
        Shuffleboard.getTab("Elevator").add("Elevator", self)
        # Shuffleboard.getTab("Elevator").add("ElevatorMech", self.mech)
        Shuffleboard.getTab("Elevator").add("ElevatorPid", self._controller)

    def periodic(self):
        # Input Logging
        FalconLogger.logInput("Elevator/MotorInput", self.__motor.get())
        FalconLogger.logInput("Elevator/MotorOutput", self.__motor.get_motor_voltage().value)
        FalconLogger.logInput("Elevator/MotorPosition_r", self.__motor.get_position().value)
        FalconLogger.logInput("Elevator/MotorVelocity_rps", self.__motor.get_velocity().value)

        FalconLogger.logInput("Elevator/EncoderPositionAbs_r", self.__encoder.get_absolute_position().value)
        FalconLogger.logInput("Elevator/EncoderPositionRel_r", self.__encoder.get_position().value)
        FalconLogger.logInput("Elevator/EncoderVelocity_rps", self.__encoder.get_velocity().value)

        # Run
        if RobotState.isDisabled():
            self.stop()
        super().periodic()

        # Visualization
        # MORE MECHANISM 2D
        #
        #

        # Output Logging
        FalconLogger.logOutput("Elevator/TargetRotation", rotationsToDegrees(self.getSetpoint()))
        FalconLogger.logOutput("Elevator/ActualRotation", rotationsToDegrees(self.getMeasurement()))

    def simulationPeriodic(self) -> None:
        # Simulation Motor Deadband
        if self.atSetpoint() and abs(self.__motor.get_duty_cycle().value) <= 0.011:
            self.__motor.set_control(self.dutyOut.with_output(0.0))

            # Motor
        self.__motor.sim_state.set_supply_voltage(RobotController.getBatteryVoltage())
        velocity = self.__motor.sim_state.motor_voltage / 12 * ElevatorConstants.KrakenSim.kMaxRps

        self.__motor.sim_state.set_rotor_velocity(velocity)
        self.__motor.sim_state.add_rotor_position(velocity * 0.02)

        # CANcoder
        self.__encoder.sim_state.set_velocity(-velocity * ElevatorConstants.kGearRatio)
        self.__encoder.sim_state.add_position(-velocity * ElevatorConstants.kGearRatio * 0.02)

    def stop(self) -> None:
        self.setSetpoint(rotationsToDegrees(self.getMeasurement()), True)

    def setSetpoint(self, setpoint: degrees, overrideRange: bool = False):
        # Limits the specific range (Protects mechanism)
        if not overrideRange:
            setpoint = min(max(setpoint, ElevatorPositions.MIN), ElevatorPositions.MAX)
        setpoint = degreesToRotations(setpoint)
        return super().setSetpoint(setpoint)

    def useOutput(self, output: float, setpoint: float) -> None:
        # Placeholder: Feed Forward Not Implemented
        feedforward = SimpleMotorFeedforwardRadians(0, 0, 0).calculate(setpoint)

        # Sets the motor speed
        self.__motor.set_control(self.dutyOut.with_output(output))

    def getMeasurement(self) -> float:
        return self.__encoder.get_absolute_position().value

    def atSetpoint(self, position: float = None) -> bool:
        if position is None or self.getSetpoint() == degreesToRotations(position):
            return self._controller.atSetpoint()
        else:
            return False