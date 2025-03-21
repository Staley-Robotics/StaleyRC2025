import typing

from commands2 import Subsystem

from wpilib import RobotState, SmartDashboard, RobotBase, Mechanism2d, Color8Bit, Color
from wpilib.simulation import SingleJointedArmSim
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor

from wpimath.units import *

from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from rev import SparkMax, SparkMaxSim, SparkMaxConfig, AlternateEncoderConfig, ClosedLoopConfig, ClosedLoopSlot, AbsoluteEncoderConfig, AbsoluteEncoder, SparkClosedLoopController
from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration

from util import FalconLogger

class CoralPivotConstants:
    gear_ratio = 25
    
    kP = 2.25 # 3
    kI = 0 #
    kD = 0.3 # 0.3
    kArbFF = 0.55 # 0.425

    kP_Coral = 2.25
    kI_Coral = 0
    kD_Coral = 0.3
    kArbFF_Coral = 0.6

    kTolerance = 1.0 # math.pi / 10
    
class CoralPivotPositions: # NOTE: these are wrong
    MIN:degrees = -74
    MAX:degrees = 90
    START:degrees = -90
    SOURCE:degrees = 5
    HOLD:degrees = 90

    Neg45:degrees = -45

    L1:degrees = 10 #Trough
    L2:degrees = -40.0
    L3:degrees = -40.0
    L4_up:degrees = 45
    L4_down:degrees = 30

class CoralManipulatorPivot(Subsystem):
    hasCoral:typing.Callable[[],bool]

    def __init__(self, motor_port:int, encoder_offset:int) -> None:
        # Special Helper Functions
        self.hasCoral:typing.Callable[[],bool] = lambda: False

        ## Init Motor
        self.motor = SparkMax( motor_port, SparkMax.MotorType.kBrushless )
        self.encoder = self.motor.getAbsoluteEncoder()
        self.controller = self.motor.getClosedLoopController()

        # config
        motorCfg = SparkMaxConfig()
        motorCfg = motorCfg.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        motorCfg = motorCfg.inverted( True )

        clCfg = ClosedLoopConfig()
        clCfg = clCfg.pid(
            CoralPivotConstants.kP,
            CoralPivotConstants.kI,
            CoralPivotConstants.kD,
            ClosedLoopSlot.kSlot0
        )
        clCfg = clCfg.pid(
            CoralPivotConstants.kP_Coral,
            CoralPivotConstants.kI_Coral,
            CoralPivotConstants.kD_Coral,
            ClosedLoopSlot.kSlot1
        )
        clCfg = clCfg.positionWrappingInputRange(0, 1).positionWrappingEnabled( True )
        clCfg = clCfg.setFeedbackSensor( ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder )

        encoderCfg = AbsoluteEncoderConfig()
        encoderCfg = encoderCfg.zeroOffset( encoder_offset )
        encoderCfg = encoderCfg.zeroCentered( True )

        # Apply Configs
        motorCfg.apply(encoderCfg)
        motorCfg.apply(clCfg)

        self.motor.configure( motorCfg, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )

        ## Mech2d
        mech = Mechanism2d( 12, 12, Color8Bit(50,50,70) )
        mechRoot = mech.getRoot( 'CoralPivot', 0.5, 0 )
        mechElevator = mechRoot.appendLigament('pretendElevator', 6, 90, color=Color8Bit( Color.kGray ) ) # replace with ref to real elevator mech
        mechStick = mechElevator.appendLigament('stick', 3, -90, color=Color8Bit( Color.kGray ))
        self.mechArmTarget = mechStick.appendLigament( 'coralPivotTarget', 4, 180, color=Color8Bit(Color.kYellow), lineWidth=4 )
        self.mechArmActual = mechStick.appendLigament( 'coralPivotActual', 8, 180, color=Color8Bit(Color.kGreen) )
        if RobotBase.isSimulation(): self.mechArmSim = mechStick.appendLigament('coralPivotSSim', 6, 180, color=Color8Bit(Color.kRed) )
        
        # Shuffleboard
        SmartDashboard.putData( 'CoralPivot', self )
        SmartDashboard.putData( 'CoralPivotMech', mech )

        ## Simulation
        self.simMotor = SparkMaxSim( self.motor, DCMotor.NEO() )
        self.simEncoder = self.simMotor.getAlternateEncoderSim()
        self.simEncoder.setPosition( degreesToRotations(-90) )

        self.armSim = SingleJointedArmSim(
            DCMotor.NEO(),
            CoralPivotConstants.gear_ratio,
            SingleJointedArmSim.estimateMOI( 0.15, 3.25 ),
            0.15,
            degreesToRadians( CoralPivotPositions.MIN ),
            degreesToRadians( CoralPivotPositions.MAX ),
            True, # Gravity
            degreesToRadians( CoralPivotPositions.START ),
        )
        self.armSim.setState( degreesToRadians( self.simEncoder.getPosition() ) , 0.0 )

    def periodic(self) -> None:
        # Logging - Write Measured Values
        FalconLogger.logInput("CoralManipulator/Pivot/MotorInput", self.motor.get())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorOutput", self.motor.getAppliedOutput())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorCurrent_a", self.motor.getOutputCurrent())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorPosition_r", self.motor.getEncoder().getPosition())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorVelocity_rpm", self.motor.getEncoder().getPosition())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorTemp_c", self.motor.getMotorTemperature())
        
        FalconLogger.logInput("CoralManipulator/Pivot/EncoderPosition_abs_r", self.encoder.getPosition())
        FalconLogger.logInput("CoralManipulator/Pivot/EncoderVelocity_c", self.encoder.getVelocity())
        
        FalconLogger.logInput("CoralManipulator/Pivot/MotorCurrent_a", self.motor.getOutputCurrent())

        # Run Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()

        # Mech2d
        self.mechArmActual.setAngle( self.getPosition() )
        self.mechArmTarget.setAngle( self.getSetpoint() )

        # Logging - Write Calculated Values
        FalconLogger.logOutput('/CoralManipulator/Pivot/TargetPosition_d', self.desiredPosition)
        FalconLogger.logOutput('/CoralManipulator/Pivot/ActualPosition_d', self.getPosition())

    def simulationPeriodic(self):
        ## Apply Simulated Data
        # self.encoder.sim_state.set_raw_position( radiansToRotations(self.armSim.getAngle()) )
        velocity_radps = self.armSim.getVelocity()
        velocity_rpm = radiansToRotations( velocity_radps ) * 60

        self.simMotor.setMotorCurrent(0)
        self.simMotor.iterate( velocity_rpm, 12, 0.02 )
        self.simMotor.getRelativeEncoderSim().iterate( velocity_rpm * CoralPivotConstants.gear_ratio, 0.02 )

        self.mechArmSim.setAngle( radiansToDegrees( self.armSim.getAngle() ) )

        # Logging
        FalconLogger.logInput("CoralManipulator/SimPivot/Velocity_rpm", velocity_rpm )
        FalconLogger.logInput("CoralManipulator/SimPivot/Position_r", radiansToRotations( self.armSim.getAngle() ) )

        ## Update MOI while you are Climbing
        # if self.hasCoral():
        #     self.simPivotArm.setInput()
        # else:
        #     self.simPivotArm.estimateMOI()

        # Simulate
        self.armSim.setInputVoltage( self.simMotor.getAppliedOutput() * self.simMotor.getBusVoltage() )
        self.armSim.update(0.02)

    def run(self) -> None:
        sp = degreesToRotations( self.getSetpoint() )

        arbFF = CoralPivotConstants.kArbFF_Coral if self.hasCoral() else CoralPivotConstants.kArbFF
        cosineScalar = math.cos( degreesToRadians( self.getPosition() ) )
        slot = ClosedLoopSlot.kSlot1 if self.hasCoral() else ClosedLoopSlot.kSlot0   

        self.controller.setReference(
            sp,
            SparkMax.ControlType.kPosition,
            slot,
            arbFF * cosineScalar,
            SparkClosedLoopController.ArbFFUnits.kVoltage
        )

    def stop(self) -> None:
        self.setSetpoint(self.getPosition())
    
    def setSetpoint(self, value:degrees, override:bool = False) -> None:
        self.desiredPosition = min(max(value, CoralPivotPositions.MIN), CoralPivotPositions.MAX)
        if override: self.desiredPosition = value

    def getSetpoint(self) -> degrees:
        return self.desiredPosition
    
    def atSetpoint(self) -> bool:
        return abs(self.getSetpoint() - self.getPosition()) < CoralPivotConstants.kTolerance

    def getPosition(self) -> degrees:
        """
        Returns the current position of the pivot in degrees (from the encoder)
        """
        val = rotationsToDegrees(self.encoder.getPosition())
        
        # if val > 180:
        #     val -= 360
        # elif val < -180:
        #     val += 360
            
        return val

    def setHasCoral(self, hasCoral:typing.Callable[[], bool]) -> None:
        self.hasCoral = hasCoral
