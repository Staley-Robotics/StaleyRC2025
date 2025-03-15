from commands2 import Subsystem

from wpilib import RobotState, SmartDashboard, RobotBase, Mechanism2d, Color8Bit, Color, XboxController
from wpilib.simulation import SingleJointedArmSim
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor
from wpimath import applyDeadband

from wpimath.units import * #radians, rotationsToRadians, rotationsToDegrees, radiansToDegrees, radiansToRotations

from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from rev import SparkMax, SparkMaxSim, SparkMaxConfig, AlternateEncoderConfig, ClosedLoopConfig, SparkClosedLoopController
from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration
from phoenix6.signals.spn_enums import SensorDirectionValue

from math import pi

from util import FalconLogger

class CoralPivotConstants:
    k_max_velocity=DCMotor.NEO550().freeSpeed * 60
    gear_ratio=25
    kP=2.5
    kI=0
    kD=0.1
    kArbFF=0.5
    tolerance:degrees = 1.0

class CoralPivotPositions: # NOTE: these are wrong
    # pi is straight forwards
    MIN:degrees = -110
    MAX:degrees = 110
    START:degrees = -90
    SOURCE:degrees = 30
    HOLD:degrees = 0.0

    L1:degrees = -45.0
    L2:degrees = -45/2.0
    L3:degrees = 0.0
    L4_up:degrees = 30
    L4_down:degrees = 60


class CoralManipulatorPivot(Subsystem):

    # Variable Declaration
    desiredPosition:degrees = 0.0

    def __init__(self, motor_port:int) -> None:
        self.driver1 = XboxController(0)
        ## Motor Init
        # using NEO 550
        self.motor = SparkMax( motor_port, SparkMax.MotorType.kBrushless )
        self.encoder = self.motor.getAlternateEncoder()
        self.controller = self.motor.getClosedLoopController()
        
        # config
        motorCfg = SparkMaxConfig()
        motorCfg = motorCfg.setIdleMode( SparkMaxConfig.IdleMode.kCoast )
        
        encCfg = AlternateEncoderConfig()
        encCfg = encCfg.countsPerRevolution(8192)

        clCfg = ClosedLoopConfig()
        clCfg = clCfg.setFeedbackSensor( ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder )
        clCfg = clCfg.pid(
            CoralPivotConstants.kP,
            CoralPivotConstants.kI,
            CoralPivotConstants.kD
        )
        clCfg = clCfg.outputRange( -1.0, 1.0 )

        motorCfg.apply( encCfg )
        motorCfg.apply( clCfg )

        self.motor.configure(
            motorCfg,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        )

        ## Encoder Init
        encoderSetpoint = CoralPivotPositions.START
        self.setSetpoint( -45.0 )
        #self.stop()

        ## Simulation
        self.simMotor = SparkMaxSim( self.motor, DCMotor.NEO550() )
        self.simMotor.setPosition( degreesToRotations( encoderSetpoint ) )
        self.simMotor.enable()

        self.simRelEncoder = self.simMotor.getRelativeEncoderSim()
        self.simRelEncoder.setPositionConversionFactor( 25 ) #1 / CoralPivotConstants.gear_ratio )
        self.simRelEncoder.setVelocityConversionFactor( 25 ) #1 / CoralPivotConstants.gear_ratio )
        self.simRelEncoder.setPosition( degreesToRotations( encoderSetpoint ) * CoralPivotConstants.gear_ratio )

        self.simAltEncoder = self.simMotor.getAlternateEncoderSim()
        self.simAltEncoder.setPositionConversionFactor( 1 ) #1 / CoralPivotConstants.gear_ratio )
        self.simAltEncoder.setVelocityConversionFactor( 1 ) #1 / CoralPivotConstants.gear_ratio )
        self.simAltEncoder.setPosition( degreesToRotations( encoderSetpoint ) )

        self.armSim = SingleJointedArmSim(
            DCMotor.NEO550(),
            CoralPivotConstants.gear_ratio,
            SingleJointedArmSim.estimateMOI( 0.30, 0.5 ), # NOTE: these are random numbers
            0.30,
            degreesToRadians( CoralPivotPositions.MIN ),
            degreesToRadians( CoralPivotPositions.MAX ),
            True,
            degreesToRadians( CoralPivotPositions.START ),
            )
        self.armSim.setState( degreesToRadians( CoralPivotPositions.START ), 0.0 )

        ## Mech2d
        self.mech = Mechanism2d( 12, 12, Color8Bit(Color.kBlack) )
        root = self.mech.getRoot( 'CoralPivot', 6, 0 )
        elevator = root.appendLigament('pretendElevator', 6, 90, color=Color8Bit(Color.kPurple) ) # replace with ref to real elevator mech
        stick = elevator.appendLigament('stick', 2, -90, color=Color8Bit(Color.kPurple))
        
        self.mechTarget = stick.appendLigament( 'armTarget', 2, 180, color=Color8Bit(Color.kRed) )
        self.mechActual = stick.appendLigament( 'armActual', 6, 180, color=Color8Bit(Color.kGreen) )
        
        if RobotBase.isSimulation():
            self.mechSim = stick.appendLigament( 'armSim', 4, 180, color=Color8Bit(Color.kYellow) )

        SmartDashboard.putData( 'Coral/mech2d', self.mech)

    # Periodic Loop
    def periodic(self) -> None:
        # Logging - Write Measured Values
        FalconLogger.logInput("Coral/Pivot/MotorInput", self.motor.get() )
        FalconLogger.logInput("Coral/Pivot/MotorOutput_v", self.motor.getAppliedOutput() )
        FalconLogger.logInput("Coral/Pivot/MotorPosition_abs_r", self.motor.getEncoder().getPosition() )
        FalconLogger.logInput("Coral/Pivot/MotorVelocity_rpm", self.motor.getEncoder().getVelocity() )
        FalconLogger.logInput("Coral/Pivot/MotorTemp_c", self.motor.getMotorTemperature() )
        FalconLogger.logInput("Coral/Pivot/MotorCurrent_a", self.motor.getOutputCurrent())

        FalconLogger.logInput("Coral/Pivot/EncoderPosition_r", self.encoder.getPosition() )
        FalconLogger.logInput("Coral/Pivot/EncoderVelocity_rpm", self.encoder.getVelocity() )

        # Run Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()

        # Mech2d
        self.mechActual.setAngle( self.getPosition() )
        self.mechTarget.setAngle( self.getSetpoint() )

        # Logging - Write Calculated Values
        FalconLogger.logOutput('CoralManipulator/Pivot/ActualPosition_d', self.getPosition() )
        FalconLogger.logOutput('CoralManipulator/Pivot/TargetPosition_d', self.getSetpoint() )

    def simulationPeriodic(self):
        ## Simulate
        FalconLogger.logInput("Coral/Pivot/SimPosition_r", self.armSim.getAngle() )
        FalconLogger.logInput("Coral/Pivot/SimPosition_d", self.armSim.getAngleDegrees() )
        FalconLogger.logInput("Coral/Pivot/SimVelocity_rps", self.armSim.getVelocity() )

        self.simMotor.setMotorCurrent( 0 )
        self.simMotor.iterate(
            radiansToRotations( self.armSim.getVelocity() ) * 60,
            self.simMotor.getBusVoltage(),
            0.020
        )
        self.simRelEncoder.iterate(
            radiansToRotations( self.armSim.getVelocity() ) * 60 * CoralPivotConstants.gear_ratio,
            0.020
        )
        # self.simAltEncoder.iterate(
        #     radiansToRotations( self.armSim.getVelocity() ) * 60,
        #     0.020
        # )        

        self.mechSim.setAngle( radiansToDegrees( self.armSim.getAngle() ) )

        appOut = self.simMotor.getAppliedOutput()
        self.armSim.setInputVoltage( appOut * self.simMotor.getBusVoltage() )
        self.armSim.update(0.02)

    def run(self) -> None:
        # x = applyDeadband( self.driver1.getLeftX(), 0.04 )
        # self.motor.set( x * 0.25 )
        self.controller.setReference(
            degreesToRotations( self.desiredPosition ),
            SparkMax.ControlType.kPosition,
            arbFeedforward = CoralPivotConstants.kArbFF * math.cos( degreesToRadians(self.getPosition()) ),
            arbFFUnits = SparkClosedLoopController.ArbFFUnits.kVoltage
        )
        return None
        val = self.controller.calculate( self.getMeasuredPosition(), self.desiredPosition )
        # print(val)
        self.motor.set( val )

    def stop(self) -> None:
        self.setSetpoint( self.getPosition(), True )

    def setSetpoint(self, value:degrees, override:bool = False) -> None:
        self.desiredPosition = min(max(value, CoralPivotPositions.MIN), CoralPivotPositions.MAX)
        if override: self.desiredPosition = value

        # self.controller.setReference(
        #     degreesToRotations( self.desiredPosition ),
        #     SparkMax.ControlType.kPosition
        # )

    def getSetpoint(self) -> degrees:
        return self.desiredPosition

    def atSetpoint(self) -> bool:
        sp = self.getSetpoint()
        cur = self.getPosition()
        return abs(sp - cur) < CoralPivotConstants.tolerance

    def getPosition(self) -> degrees:
        return rotationsToDegrees( self.encoder.getPosition() )

