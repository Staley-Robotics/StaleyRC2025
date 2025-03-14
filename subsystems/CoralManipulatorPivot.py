from commands2 import Subsystem

from wpilib import RobotState, SmartDashboard, RobotBase, Mechanism2d, Color8Bit
from wpilib.simulation import SingleJointedArmSim
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor

from wpimath.units import radians, rotationsToRadians, rotationsToDegrees, radiansToDegrees, radiansToRotations

from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from rev import SparkMax, SparkMaxSim, SparkMaxConfig, AlternateEncoderConfig, ClosedLoopConfig, ClosedLoopSlot
from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration

from math import pi

from util import FalconLogger

class CoralManipulatorPivot(Subsystem):

    class PivotConstants:
        k_max_velocity=DCMotor.NEO550().freeSpeed * 60
        gear_ratio=1/25
        kP=1.75
        kI=0
        kD=0.1
        kFF=0.0
    
    class PivotPositions: # NOTE: these are wrong
        # pi is straight forwards
        MIN:radians = -pi/2
        MAX:radians = pi/2
        START:radians = pi
        SOURCE:radians = pi/4
        HOLD:radians = pi/2

        L1:radians = -pi/6 #Trough
        L2:radians = 7*pi/6
        L3:radians = 7*pi/6
        L4_up:radians = 2*pi/3
        L4_down:radians = 7*pi/6

    # Variable Declaration

    tolerance = pi / 10 #ntproperty("/CoralManipulatorPivot/AtPostionTolerance", pi/20, persistent=True)


    def __init__(self, motor_port:int, encoder_offset:int) -> None:

        ## Motor
        self.pivotMotor = SparkMax( motor_port, SparkMax.MotorType.kBrushless )
        # config
        motorConfig = SparkMaxConfig()
        motorConfig = motorConfig.setIdleMode( SparkMaxConfig.IdleMode.kCoast )

        encoderConfig = AlternateEncoderConfig()
        encoderConfig = encoderConfig.inverted( False )
        # encoderConfig = encoderConfig.zeroOffset(encoder_offset)

        closedLoopConfig = ClosedLoopConfig()
        closedLoopConfig = closedLoopConfig.pidf(
            self.PivotConstants.kP,
            self.PivotConstants.kI,
            self.PivotConstants.kD,
            self.PivotConstants.kFF,
            ClosedLoopSlot.kSlot0
        )
        closedLoopConfig = closedLoopConfig.positionWrappingInputRange(-pi, pi).positionWrappingEnabled( True )
        closedLoopConfig = closedLoopConfig.setFeedbackSensor( ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder )

        motorConfig.apply(encoderConfig)
        motorConfig.apply(closedLoopConfig)

        self.pivotMotor.configure( motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )

        ## Encoder
        self.encoder = self.pivotMotor.getAlternateEncoder()

        ## Control
        self.controller = self.pivotMotor.getClosedLoopController()

        self.desiredPosition: radians = self.getPosition()

        ## Mech2d
        self.mech = Mechanism2d( 12, 12, Color8Bit(50,50,70) )
        root = self.mech.getRoot( 'CoralPivot', 6, 0 )
        elevator = root.appendLigament('pretendElevator', 6, 90, color=Color8Bit(200,200,170) ) # replace with ref to real elevator mech
        stick = elevator.appendLigament('stick', 3, -90, color=Color8Bit(200,200,170))
        self.pivotArm = stick.appendLigament( 'arm', 6, 180, color=Color8Bit(20,240,20) )
        self.guide = stick.appendLigament( 'guidearm', 3, 180, color=Color8Bit(20,170,20) )
        
        SmartDashboard.putData( '/CoralManipulatorPivot/mech2d', self.mech)

        ## Simulation
        if RobotBase.isSimulation():
            gearbox = DCMotor.NEO550()
            self.armSim = SingleJointedArmSim(
                gearbox,
                1/self.PivotConstants.gear_ratio,
                SingleJointedArmSim.estimateMOI( 0.34, 1. ), # NOTE: these are random numbers
                0.15,
                self.PivotPositions.MIN,
                self.PivotPositions.MAX,
                True,
                self.PivotPositions.START,
             )
            
            self.simMotor = SparkMaxSim( self.pivotMotor, DCMotor.NEO550() )
            self.simEncoder = self.simMotor.getAlternateEncoderSim()
            # self.simEncoder.setPositionConversionFactor(pi/2)

    def periodic(self) -> None:
        # Logging - Write Measured Values
        FalconLogger.logInput('/CoralManipulatorPivot/MeasuredPositionRadians', self.getPosition())

        # Run Subsystem
        if RobotState.isDisabled():
            self.stop()
        # No run bc hardware PID
        
        # Mech2d
        self.pivotArm.setAngle( radiansToDegrees(self.getPosition()) )
        self.guide.setAngle( radiansToDegrees(self.getSetpoint()) )
        
        # Logging - Write Calculated Values
        FalconLogger.logOutput('/CoralManipulatorPivot/DesiredPosition', self.desiredPosition)
    
    def simulationPeriodic(self):
        # Simulate
        self.armSim.setInputVoltage( self.simMotor.getAppliedOutput() * self.simMotor.getBusVoltage() )
        self.armSim.update(0.02)

        ## Apply Simulated Data
        # self.encoder.sim_state.set_raw_position( radiansToRotations(self.armSim.getAngle()) )
        
        self.simMotor.iterate(
            self.armSim.getVelocity() * CoralManipulatorPivot.PivotConstants.gear_ratio,
            12,
            0.02
        )

        self.pivotArm.setAngle( radiansToDegrees(self.armSim.getAngle()) )

    def stop(self) -> None:
        self.setSetpoint(self.getPosition())
        self.desiredPosition = self.getPosition()

    def setSetpoint(self, value:radians) -> None:
        self.controller.setReference(
            min(max(value, CoralManipulatorPivot.PivotPositions.MIN), CoralManipulatorPivot.PivotPositions.MAX),
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
            )

    def getSetpoint(self) -> radians:
        return self.desiredPosition
    
    def atSetpoint(self) -> bool:
        return abs(self.getSetpoint() - self.getPosition()) < self.tolerance # TODO: figure out tolerance

    def getPosition(self) -> radians:
        return rotationsToRadians(self.encoder.getPosition())
