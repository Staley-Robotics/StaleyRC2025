import math
from enum import Enum


from commands2 import Subsystem
from wpilib import SmartDashboard, RobotBase, RobotState, Mechanism2d, Color8Bit, Color, I2C, DigitalInput
from wpilib.shuffleboard import Shuffleboard
from wpilib.simulation import SingleJointedArmSim
from wpimath.system.plant import DCMotor
from wpimath.units import *

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import VoltageOut, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue, SensorDirectionValue

from phoenix5 import WPI_TalonSRX, TalonSRX, TalonSRXControlMode, NeutralMode

from rev import SparkMax, SparkMaxConfig, SparkMaxSim, SparkBase, ClosedLoopConfig, SparkClosedLoopController, ClosedLoopSlot, AbsoluteEncoderConfig, ColorSensorV3

from util import FalconLogger

class ColorConstants:
    red = 44
    green = 133
    blue = 75
    tolerence = 5 # if red is 44 and tolerance is 5, robot will see correct red value if red is in range (39, 49) exclusive
    proximity = 75

class AlgaeManipulatorPositions:
    MAX = 100.0
    HOLD = 85.0
    PLACE = 60.0
    GRAB = 35.0
    MIN = 10.0

class AlgaeIntakeState(Enum):
    IN = -0.5
    OUT = 0.5
    OFF = 0

class AlgaeManipulatorConstants:
    # Pivot Constants
    pivot_kP: float = 0.75
    pivot_kI: float = 0.00002
    pivot_kD: float = 0.05
    pivot_kFF: float = 0.0
    pivot_kArbFF: float = 0.97006
    pivot_kV: float = 0.0
    pivot_kTolerance: float = 1

    pivot_kP_Algae: float = 2.2
    pivot_kI_Algae: float = 0.0
    pivot_kD_Algae: float = 0.0
    pivot_kFF_Algae: float = 0.0
    pivot_kArbFF_Algae: float = 1.0

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
    pivotSetpoint: float = 0.0

    __intakeMotor: TalonSRX = None
    intakeState = AlgaeIntakeState.OFF

    __kMotorOffset = 0.5902802

    def __init__(self):
        ## Init Motors
        self.__leadMotor = SparkMax(1, SparkMax.MotorType.kBrushless)
        self.__followMotor = SparkMax(2, SparkMax.MotorType.kBrushless)

        self.__pivotEncoder = self.__leadMotor.getAbsoluteEncoder()
        self.__pivotController = self.__leadMotor.getClosedLoopController()
        
        # Init motors and config
        lMotorCfg = SparkMaxConfig()
        lMotorCfg = lMotorCfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )

        fMotorCfg = SparkMaxConfig()
        fMotorCfg = fMotorCfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        # fMotorCfg = fMotorCfg.inverted(True)
        fMotorCfg = fMotorCfg.follow(self.__leadMotor.getDeviceId(), True)

        clConfig = ClosedLoopConfig()
        clConfig = clConfig.pidf(
            AlgaeManipulatorConstants.pivot_kP,
            AlgaeManipulatorConstants.pivot_kI,
            AlgaeManipulatorConstants.pivot_kD,
            AlgaeManipulatorConstants.pivot_kFF,
            ClosedLoopSlot.kSlot0
        )
        clConfig = clConfig.pidf(
            AlgaeManipulatorConstants.pivot_kP_Algae,
            AlgaeManipulatorConstants.pivot_kI_Algae,
            AlgaeManipulatorConstants.pivot_kD_Algae,
            AlgaeManipulatorConstants.pivot_kFF_Algae,
            ClosedLoopSlot.kSlot1
        )
        clConfig = clConfig.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        clConfig = clConfig.positionWrappingEnabled(True).positionWrappingInputRange(0, 1)

        encConfig = AbsoluteEncoderConfig()
        encConfig = encConfig.zeroOffset(self.__kMotorOffset)
        encConfig = encConfig.zeroCentered(True)

        # Apply configs
        lMotorCfg.apply(clConfig)
        lMotorCfg.apply(encConfig)

        self.__leadMotor.configure( lMotorCfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters )
        self.__followMotor.configure( fMotorCfg, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters )

        # 775pro
        self.__intakeMotor = WPI_TalonSRX(31)
        self.__intakeMotor.setNeutralMode(NeutralMode.Brake)

        # Algae Detection Sensor
        # self.__irBeam = DigitalInput(3)
        if RobotBase.isSimulation(): self.__irBeam = DigitalInput(3)
        self.__colorSensor = ColorSensorV3(I2C.Port(0))

        # Mechanism 2d stuff
        mech = Mechanism2d(30, 40, Color8Bit(50,50,70))
        mechRoot = mech.getRoot("Pivot", 15, 0 )
        mechPost = mechRoot.appendLigament( "AlgaePost", 6, 90, 2, Color8Bit( Color.kGray ) )
        self.mechAlgaeTarget = mechPost.appendLigament("AlgaeTarget", 14, 0, 2, color=Color8Bit( Color.kYellow ) )
        self.mechAlgaeActual = mechPost.appendLigament("AlgaeActual", 18, 0, 3, color=Color8Bit( Color.kGreen ) )
        if RobotBase.isSimulation(): self.mechAlgaeSim = mechPost.appendLigament("AlgaeSSim", 16, 0, 3, color=Color8Bit( Color.kRed ) )

        # Shuffleboard
        SmartDashboard.putData( "Algae", self )
        SmartDashboard.putData( "AlgaeMech", mech )
        #Shuffleboard.getTab("AlgaeManipulator").add("AlgaeManipulator", self)

        # Pivot Simulation
        self.simLMotor = SparkMaxSim(self.__leadMotor, DCMotor.NEO(1))
        #self.simLMotor.setPosition( degreesToRotations( 90 ) )

        self.simPivotArm = SingleJointedArmSim(
            DCMotor.NEO550(2),
            AlgaeManipulatorConstants.pivot_kGearRatio,
            SingleJointedArmSim.estimateMOI( 0.50, 2.2 ), # NOTE: these are random numbers
            0.50,
            degreesToRadians( -15.0 ),
            degreesToRadians( 100.0 ),
            True, #Gravity
            degreesToRadians( 90.0 ),
        )
        self.simPivotArm.setState( self.getMeasurement(), 0.0 )

        # Intake Simulation
        self.simIntake = self.__intakeMotor.getSimCollection()

    def periodic(self) -> None:
        # Input Logging - Neo - Pivot
        FalconLogger.logInput("AlgaeManipulator/Pivot/LeadMotorInput", self.__leadMotor.get())
        FalconLogger.logInput("AlgaeManipulator/Pivot/LeadMotorOutput", self.__leadMotor.getAppliedOutput())
        FalconLogger.logInput("AlgaeManipulator/Pivot/LeadMotorCurrent_a", self.__leadMotor.getOutputCurrent())
        FalconLogger.logInput("AlgaeManipulator/Pivot/LeadMotorPosition_r", self.__leadMotor.getEncoder().getPosition())
        FalconLogger.logInput("AlgaeManipulator/Pivot/LeadMotorVelocity_rpm", self.__leadMotor.getEncoder().getVelocity())
        FalconLogger.logInput("AlgaeManipulator/Pivot/LeadMotorTemp_c", self.__leadMotor.getMotorTemperature())

        FalconLogger.logInput("AlgaeManipulator/Pivot/FollowMotorInput", self.__followMotor.get())
        FalconLogger.logInput("AlgaeManipulator/Pivot/FollowMotorOutput", self.__followMotor.getAppliedOutput())
        FalconLogger.logInput("AlgaeManipulator/Pivot/FollowMotorCurrent_a", self.__followMotor.getOutputCurrent())
        FalconLogger.logInput("AlgaeManipulator/Pivot/FollowMotorPosition_r", self.__followMotor.getEncoder().getPosition())
        FalconLogger.logInput("AlgaeManipulator/Pivot/FollowMotorVelocity_rpm", self.__followMotor.getEncoder().getVelocity())
        FalconLogger.logInput("AlgaeManipulator/Pivot/FollowMotorTemp_c", self.__followMotor.getMotorTemperature())

        FalconLogger.logInput("AlgaeManipulator/Pivot/EncoderPosition_r", self.__pivotEncoder.getPosition())
        FalconLogger.logInput("AlgaeManipulator/Pivot/EncoderVelocity_rpm", self.__pivotEncoder.getVelocity())

        # Input Logging - 775pro - Intake
        FalconLogger.logInput("AlgaeManipulator/Intake/MotorOutputPercent", self.__intakeMotor.getMotorOutputPercent())
        FalconLogger.logInput("AlgaeManipulator/Intake/MotorVoltage", self.__intakeMotor.getMotorOutputVoltage())

        # Input Logging - Algae Sensor
        FalconLogger.logInput("AlgaeManipulator/Sensor/Proximity", self.__colorSensor.getProximity() )
        FalconLogger.logInput("AlgaeManipulator/Sensor/Color.red", self.__colorSensor.getColor().red )
        FalconLogger.logInput("AlgaeManipulator/Sensor/Color.blue", self.__colorSensor.getColor().blue )
        FalconLogger.logInput("AlgaeManipulator/Sensor/Color.green", self.__colorSensor.getColor().green )

        # Run
        if RobotState.isDisabled():
            self.stop()
        else:
            # Intake
            self.run()

        self.mechAlgaeActual.setAngle( self.getMeasurement() - 90.0 )
        self.mechAlgaeTarget.setAngle( self.getSetpoint() - 90.0 )

        # Output Logging
        FalconLogger.logOutput("AlgaeManipulator/Pivot/TargetAngle_d", self.getSetpoint())
        FalconLogger.logOutput("AlgaeManipulator/Pivot/ActualAngle_d", self.getMeasurement())
        FalconLogger.logOutput("AlgaeManipulator/Intake/TargetSpeed_p", self.intakeState.value)
        FalconLogger.logOutput("AlgaeManipulator/Intake/ActualSpeed_p", self.__intakeMotor.getMotorOutputPercent())
        FalconLogger.logOutput("AlgaeManipulator/HasAlgae", self.hasAlgae() )

    def simulationPeriodic(self) -> None:
        FalconLogger.logInput("AlgaeManipulator/IRBeam", self.__irBeam.get())

        # Neo Periodic
        #driveRpm = AlgaeManipulatorConstants.NeoSim.kMaxRpm * self.__leadMotor.getAppliedOutput()
        driveRadps = self.simPivotArm.getVelocity()
        driveRpm = radiansToRotations( driveRadps ) * 60

        self.simLMotor.setMotorCurrent(0)
        self.simLMotor.iterate( driveRpm , 12, 0.02)
        self.simLMotor.getRelativeEncoderSim().iterate( driveRpm * AlgaeManipulatorConstants.pivot_kGearRatio, 0.02 )
        #self.simPivot.getAbsoluteEncoderSim().iterate(driveRpm / AlgaeManipulatorConstants.pivot_kGearRatio, 0.02)

        # self.simPivot.getRelativeEncoderSim().setVelocity(driveRpm)
        # self.simPivot.getAbsoluteEncoderSim().setVelocity(driveRpm)
        # self.simPivot.getAbsoluteEncoderSim().setPosition(self.simPivot.getRelativeEncoderSim().getPosition() % 1)

        # 775pro Periodic
        velocity = AlgaeManipulatorConstants.Vex775Sim.kMaxRpm * self.__intakeMotor.getMotorOutputPercent()
        velocity_Tp100ms = int( velocity * 2048 / 60 / 10 ) # RPM * Tickzzzzxxs/Rot * 1 M/60sec * 1 sec / 10 (100ms)
        self.simIntake.setAnalogVelocity(velocity_Tp100ms)
        self.simIntake.addQuadraturePosition( int( velocity_Tp100ms * 0.02 ) )

        self.mechAlgaeSim.setAngle( self.simPivotArm.getAngleDegrees() - 90.0 )

        ## Update MOI when you have Algae?
        # if self.hasAlgae():
        #     self.simPivotArm.setInput()
        # else:
        #     self.simPivotArm.estimateMOI()

        self.simPivotArm.setInputVoltage( self.__leadMotor.getAppliedOutput() * 12.0 )
        self.simPivotArm.update( 0.02 )

    def run(self) -> None:
        # Pivot
        sp = degreesToRotations( self.getSetpoint() )

        arbFF = AlgaeManipulatorConstants.pivot_kArbFF_Algae if self.hasAlgae() else AlgaeManipulatorConstants.pivot_kArbFF
        cosineScalar = math.cos( degreesToRadians( self.getMeasurement() ) )
        slot = ClosedLoopSlot.kSlot1 if self.hasAlgae() else ClosedLoopSlot.kSlot0

        self.__pivotController.setReference(
            sp,
            SparkBase.ControlType.kPosition,
            slot,
            arbFF * cosineScalar,
            SparkClosedLoopController.ArbFFUnits.kVoltage
        )

        # Intake
        self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, self.intakeState.value)

    def stop(self) -> None:
        """
        Stops the manipulator from moving and the intake from rotating
        """
        self.setSetpoint(self.getMeasurement(), True)
        self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)

    def setSetpoint(self, setpoint: degrees, override: bool = False):
        """
        Sets the setpoint for the manipulator, accepts degrees
        """
        # Limits the specific range (Protects mechanism)
        if not override:
            setpoint = min(max(setpoint, AlgaeManipulatorPositions.MIN), AlgaeManipulatorPositions.MAX)
        self.pivotSetpoint = setpoint


    def getMeasurement(self) -> float:
        """
        Returns the current position of the manipulator in degrees
        """
        measurement = rotationsToDegrees(self.__pivotEncoder.getPosition())
        # if measurement > 180:
        #     measurement -= 360
        # if measurement < -180:
        #     measurement += 360
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

    def setIntake(self, state: AlgaeIntakeState):
        """
        Sets the state of the intake
        """
        self.intakeState = state
        self.__intakeMotor.set(TalonSRXControlMode.PercentOutput, state.value)

    def hasAlgae(self) -> bool:
        """
        Returns if the intake has a ball
        """
        if RobotBase.isSimulation():
            return not self.__irBeam.get()
        else:
            def inRange(actual, target, tolerance):
                return abs( target - actual ) < tolerance

            color = self.__colorSensor.getColor()
            inProximity = self.__colorSensor.getProximity() > ColorConstants.proximity
            inRangeRed = inRange( color.red * 255, ColorConstants.red, ColorConstants.tolerence )
            inRangeGreen = inRange( color.green * 255, ColorConstants.green, ColorConstants.tolerence )
            inRangeBlue = inRange( color.blue * 255, ColorConstants.blue, ColorConstants.tolerence )

            return inProximity and inRangeRed and inRangeGreen and inRangeBlue
