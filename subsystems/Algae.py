import math

from commands2 import Subsystem
from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, RobotController, Victor, DigitalInput, Color, I2C, LEDPattern
from wpilib.shuffleboard import Shuffleboard
from wpimath.controller import PIDController, SimpleMotorFeedforwardRadians
from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations, degrees, degreesToRotations, rotationsToDegrees, kSecondsPerMinute

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, \
    SensorDirectionValue

from phoenix5 import TalonSRX, TalonSRXControlMode, NeutralMode

from rev import SparkMax, SparkMaxConfig, SparkMaxSim, SparkBase, ClosedLoopConfig, AbsoluteEncoderConfig, ColorSensorV3

from enum import Enum

from util import FalconLogger

class ColorSensorConstants:
    RED = 44
    GREEN = 133
    BLUE = 75
    COLOR_TOLERANCE = 5 # if red is 44 and tolerance is 5, robot will see correct red value if red is in range (39, 49) exclusive
    MIN_PROXIMITY = 75
    

class AlgaeManipulatorPositions:
    MAX = 90.0
    HOLD = 80.0
    PLACE = 70.0
    GRAB = 35.0
    MIN = 0.0


class IntakeState(Enum):
    IN = -0.5
    OUT = 0.5
    OFF = 0


class AlgaeManipulatorConstants:
    # Pivot Constants
    pivot_kP: float = 2
    pivot_kI: float = 0.0
    pivot_kD: float = 0.0
    pivot_kFF: float = 0.0
    pivot_kV: float = 0.0
    pivot_kTolerance: float = 0.01

    pivot_kGearRatio: float = 8
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
    """
    The AlgaeManipulator subsystem is responsible for controlling the pivot and intake of the robot where Algae is concerned
    """
    # Motors (Neo and 775pro, at 25:1 and 10.3333:1 respectively)
    __pivotMotorOne: SparkMax = None
    __pivotMotorTwo: SparkMax = None
    pivotSetpoint: float = None

    __intakeMotor: TalonSRX = None
    intakeState = IntakeState.OFF

    # __irBeam: DigitalInput = None

    __colorSensor: ColorSensorV3 = None

    __kMotorOffset = 0.80742

    def __init__(self):
        # Motor
        # Neo
        self.__pivotMotorOne = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.__pivotMotorTwo = SparkMax(2, SparkMax.MotorType.kBrushless)

        self.__pivotController = self.__pivotMotorOne.getClosedLoopController()
        
        self.sparkConfig = SparkMaxConfig()

        self.clConfig = ClosedLoopConfig()
        self.clConfig = self.clConfig.pidf(AlgaeManipulatorConstants.pivot_kP, AlgaeManipulatorConstants.pivot_kI,
                                         AlgaeManipulatorConstants.pivot_kD, AlgaeManipulatorConstants.pivot_kFF)
        self.clConfig = self.clConfig.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.clConfig = self.clConfig.positionWrappingEnabled(True).positionWrappingInputRange(0, 1)

        self.absEncoderConfig = AbsoluteEncoderConfig()
        self.absEncoderConfig = self.absEncoderConfig.zeroOffset(self.__kMotorOffset)

        self.sparkConfig.apply(self.clConfig)
        self.sparkConfig.apply(self.absEncoderConfig)
        self.sparkConfig.inverted(True)


        self.__pivotMotorOne.configure(self.sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                                       SparkBase.PersistMode.kNoPersistParameters)

        self.sparkConfig.inverted(False)
        self.sparkConfig.follow(self.__pivotMotorOne.getDeviceId(), True)


        self.__pivotMotorTwo.configure(self.sparkConfig, SparkBase.ResetMode.kNoResetSafeParameters,
                                       SparkBase.PersistMode.kNoPersistParameters)




        # 775pro
        self.__intakeMotor = TalonSRX(31)
        self.__intakeMotor.setNeutralMode(NeutralMode.Brake)
        # Zero the motor
        self.voltOut = VoltageOut(0, use_timesync=True)
        self.dutyOut = DutyCycleOut(0, use_timesync=True)

        # Mechanism 2d stuff
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

        # IR Beam
        # self.__irBeam = DigitalInput(3)

        # Color Sensor
        self.__colorSensor = ColorSensorV3(I2C.Port(0))
        self.color = self.__colorSensor.getColor()

        # Shuffleboard
        Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulator", self)

    def periodic(self) -> None:
        # Input Logging - Neo - Pivot
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorInput", self.__pivotMotorOne.get())
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorOutput", self.__pivotMotorOne.getAppliedOutput())
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorPosition_abs_r", self.__pivotMotorOne.getAbsoluteEncoder().getPosition())
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorTemp_c", self.__pivotMotorOne.getMotorTemperature())
        FalconLogger.logInput("AlgaeManipulator/Pivot/MotorCurrent_a", self.__pivotMotorOne.getOutputCurrent())

        # Input Logging - 775pro - Intake
        FalconLogger.logInput("AlgaeManipulator/Intake/MotorOutputPercent", self.__intakeMotor.getMotorOutputPercent())
        FalconLogger.logInput("AlgaeManipulator/Intake/MotorVoltage", self.__intakeMotor.getMotorOutputVoltage())

        # Input Logging - IR Beam
        FalconLogger.logInput("AlgaeManipulator/IRBeam", self.__irBeam.get())

        # Run
        if RobotState.isDisabled():
            self.stop()
        else:
            # Intake
            self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, self.intakeState.value)

        self.manipulator.setAngle(self.getMeasurement())

        # Output Logging
        FalconLogger.logOutput("AlgaeManipulator/Pivot/TargetAngle_d", self.getSetpoint())
        FalconLogger.logOutput("AlgaeManipulator/Pivot/ActualAngle_d", self.getMeasurement())

        # Output Loggin
        FalconLogger.logOutput("AlgaeManipulator/Intake/DesiredState", self.intakeState.value)
        FalconLogger.logOutput("AlgaeManipulator/Intake/CurrentState", self.__intakeMotor.getMotorOutputPercent())
        FalconLogger.logOutput("AlgaeManipulator/Intake/Voltage", self.__intakeMotor.getMotorOutputVoltage())

    def simulationPeriodic(self) -> None:
        # Neo Periodic
        driveRpm = AlgaeManipulatorConstants.NeoSim.kMaxRpm * self.__pivotMotorOne.getAppliedOutput()

        self.simPivot.setMotorCurrent(0)
        self.simPivot.iterate(driveRpm, 12, 0.02)
        self.simPivot.getRelativeEncoderSim().iterate(driveRpm, 0.02)
        self.simPivot.getAbsoluteEncoderSim().iterate(driveRpm / AlgaeManipulatorConstants.pivot_kGearRatio, 0.02)

        # self.simPivot.getRelativeEncoderSim().setVelocity(driveRpm)
        # self.simPivot.getAbsoluteEncoderSim().setVelocity(driveRpm)
        # self.simPivot.getAbsoluteEncoderSim().setPosition(self.simPivot.getRelativeEncoderSim().getPosition() % 1)

        # 775pro Periodic
        velocity = AlgaeManipulatorConstants.Vex775Sim.kMaxRpm * self.__intakeMotor.getMotorOutputPercent()
        self.simIntake.sim_state.set_rotor_velocity(velocity)
        self.simIntake.sim_state.add_rotor_position(velocity * 0.02)

    def stop(self) -> None:
        """
        Stops the manipulator from moving and the intake from rotating
        """
        self.setSetpoint(self.getMeasurement(), True)
        self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)

    def setSetpoint(self, setpoint: degrees, overrideRange: bool = False):
        """
        Sets the setpoint for the manipulator, accepts degrees
        """
        # Limits the specific range (Protects mechanism)
        if not overrideRange:
            setpoint = min(max(setpoint, AlgaeManipulatorPositions.MIN), AlgaeManipulatorPositions.MAX)
        self.pivotSetpoint = setpoint
        self.__pivotController.setReference(degreesToRotations(setpoint), SparkBase.ControlType.kPosition)

    def getMeasurement(self) -> float:
        """
        Returns the current position of the manipulator in degrees
        """
        measurement = rotationsToDegrees(self.__pivotMotorOne.getAbsoluteEncoder().getPosition())
        if measurement > 180:
            measurement -= 360
        if measurement < -180:
            measurement += 360
        return measurement

    def atSetpoint(self, positionDeg: float = None) -> bool:
        """
        Returns if the manipulator is at the setpoint
        """
        if positionDeg is None:
            positionDeg = self.getMeasurement()
        return self.pivotSetpoint == positionDeg

    def getSetpoint(self) -> float:
        """
        Returns the current setpoint of the manipulator in degrees
        """
        return self.pivotSetpoint

    def setIntake(self, state: IntakeState):
        """
        Sets the state of the intake, uses the
        """
        self.intakeState = state
        self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, state.value)

    def hasAlgae(self) -> bool:
        """
        Returns if the intake has a ball
        """
        self.color = self.__colorSensor.getColor()
        red_min = ColorSensorConstants.RED - ColorSensorConstants.COLOR_TOLERANCE
        red_max = ColorSensorConstants.RED + ColorSensorConstants.COLOR_TOLERANCE
        green_min = ColorSensorConstants.GREEN - ColorSensorConstants.COLOR_TOLERANCE
        green_max = ColorSensorConstants.GREEN + ColorSensorConstants.COLOR_TOLERANCE
        blue_min = ColorSensorConstants.BLUE - ColorSensorConstants.COLOR_TOLERANCE
        blue_max = ColorSensorConstants.BLUE + ColorSensorConstants.COLOR_TOLERANCE

        return self.__colorSensor.getProximity() > ColorSensorConstants.MIN_PROXIMITY\
            and self.is_within_range(self.color.red * 255, red_min, red_max)\
                and self.is_within_range(self.color.green * 255, green_min, green_max)\
                    and self.is_within_range(self.color.blue * 255, blue_min, blue_max)\
            
        
    
    def is_within_range(self, val, min, max):
        '''Returns whether val is in range (min, max) exclusive'''
        return val < max and val > min