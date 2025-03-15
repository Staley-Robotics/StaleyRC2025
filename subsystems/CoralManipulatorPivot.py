from commands2 import Subsystem

from wpilib import RobotState, SmartDashboard, RobotBase, Mechanism2d, Color8Bit, Color
from wpilib.simulation import SingleJointedArmSim
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor

from wpimath.units import radians, rotationsToRadians, rotationsToDegrees, radiansToDegrees, radiansToRotations, \
    degrees, degreesToRotations, degreesToRadians

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
        gear_ratio=25
        kP=1
        kI=0
        kD=0
        kFF=0.0
    
    class PivotPositions: # NOTE: these are wrong
        MIN:degrees = -110
        MAX:degrees = 110
        START:degrees = -90
        SOURCE:degrees = 45
        HOLD:degrees = 90

        L1:degrees = -80 #Trough
        L2:degrees = 0
        L3:degrees = 15
        L4_up:degrees = 45
        L4_down:degrees = 30

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
        encoderConfig = encoderConfig.positionConversionFactor(1/25).velocityConversionFactor(1/25)
        encoderConfig = encoderConfig.countsPerRevolution(8192)
        # encoderConfig = encoderConfig.zeroOffset(encoder_offset)

        closedLoopConfig = ClosedLoopConfig()
        closedLoopConfig = closedLoopConfig.pidf(
            self.PivotConstants.kP,
            self.PivotConstants.kI,
            self.PivotConstants.kD,
            self.PivotConstants.kFF,
            ClosedLoopSlot.kSlot0
        )
        closedLoopConfig = closedLoopConfig.positionWrappingInputRange(-180, 180).positionWrappingEnabled( True )
        closedLoopConfig = closedLoopConfig.setFeedbackSensor( ClosedLoopConfig.FeedbackSensor.kAlternateOrExternalEncoder )

        motorConfig.apply(encoderConfig)
        motorConfig.apply(closedLoopConfig)

        self.pivotMotor.configure( motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )

        ## Encoder
        self.encoder = self.pivotMotor.getAlternateEncoder()

        self.encoder.setPosition(degreesToRotations(-90))

        ## Control
        self.controller = self.pivotMotor.getClosedLoopController()

        self.desiredPosition: degrees = self.getPosition()

        ## Mech2d
        self.mech = Mechanism2d( 12, 12, Color8Bit(50,50,70) )
        root = self.mech.getRoot( 'CoralPivot', 6, 0 )
        elevator = root.appendLigament('pretendElevator', 6, 90, color=Color8Bit(200,200,170) ) # replace with ref to real elevator mech
        stick = elevator.appendLigament('stick', 3, -90, color=Color8Bit(200,200,170))
        self.pivotArm = stick.appendLigament( 'arm', 6, 180, color=Color8Bit(Color.kGreen) )
        self.guide = stick.appendLigament( 'guideArm', 4, 180, color=Color8Bit(Color.kBlue) )
        self.pivotSim = stick.appendLigament('simArm', 2, 180, color=Color8Bit(Color.kRed) )
        
        SmartDashboard.putData( '/CoralManipulatorPivot/mech2d', self.mech)

        ## Simulation
        if RobotBase.isSimulation():
            gearbox = DCMotor.NEO550()
            self.armSim = SingleJointedArmSim(
                gearbox,
                self.PivotConstants.gear_ratio,
                SingleJointedArmSim.estimateMOI( 0.15, 0.5 ), # NOTE: these are random numbers
                0.15,
                degreesToRadians( self.PivotPositions.MIN ),
                degreesToRadians( self.PivotPositions.MAX ),
                True,
                degreesToRadians( self.PivotPositions.START ),
             )
            
            self.simMotor = SparkMaxSim( self.pivotMotor, DCMotor.NEO550() )
            #self.simMotor.setPosition( degreesToRotations(-90) * 25 )

            self.simEncoder = self.simMotor.getAlternateEncoderSim()
            # self.simEncoder.setPositionConversionFactor(1 / 25)
            # self.simEncoder.setVelocityConversionFactor(1 / 25)
            self.simEncoder.setPosition( degreesToRotations(-90) )

            # self.simEncoder.setPositionConversionFactor(pi/2)

    def periodic(self) -> None:
        # Logging - Write Measured Values
        FalconLogger.logInput("CoralManipulator/Pivot/MotorInput", self.pivotMotor.get())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorOutput", self.pivotMotor.getAppliedOutput())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorPosition_abs_r", self.encoder.getPosition())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorTemp_c", self.pivotMotor.getMotorTemperature())
        FalconLogger.logInput("CoralManipulator/Pivot/MotorCurrent_a", self.pivotMotor.getOutputCurrent())

        # Run Subsystem
        if RobotState.isDisabled():
            # self.stop()
            pass
        # No run bc hardware PID

        # Mech2d
        self.pivotArm.setAngle( self.getPosition() )
        self.guide.setAngle( self.getSetpoint() )

        # Logging - Write Calculated Values
        FalconLogger.logOutput('/CoralManipulator/Pivot/TargetPosition_d', self.desiredPosition)
        FalconLogger.logOutput('/CoralManipulator/Pivot/ActualPosition_d', self.getPosition())

    def simulationPeriodic(self):
        # Simulate
        self.armSim.setInputVoltage( self.simMotor.getAppliedOutput() * self.simMotor.getBusVoltage() )
        self.armSim.update(0.02)

        # Logging
        FalconLogger.logInput("CoralManipulator/SimPivot/MotorVelocity_radiansPerSecond", self.armSim.getVelocity())
        FalconLogger.logInput("CoralManipulator/SimPivot/MotorPosition_degrees", radiansToDegrees( self.armSim.getAngle() ) )

        ## Apply Simulated Data
        # self.encoder.sim_state.set_raw_position( radiansToRotations(self.armSim.getAngle()) )
        
        self.simMotor.iterate(
            radiansToRotations(self.armSim.getVelocity()) * CoralManipulatorPivot.PivotConstants.gear_ratio,
            12,
            0.02
        )
        self.simEncoder.iterate(
            radiansToRotations(self.armSim.getVelocity()),
            0.02
        )

        self.pivotSim.setAngle( radiansToDegrees( self.armSim.getAngle() ) )

    def stop(self) -> None:
        self.setSetpoint(self.getPosition())

    def setSetpoint(self, value:degrees) -> None:
        self.desiredPosition = min(max(value, CoralManipulatorPivot.PivotPositions.MIN), CoralManipulatorPivot.PivotPositions.MAX)
        self.controller.setReference(
            degreesToRotations(self.desiredPosition),
            SparkMax.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
            )

    def getSetpoint(self) -> degrees:
        return self.desiredPosition
    
    def atSetpoint(self) -> bool:
        return abs(self.getSetpoint() - self.getPosition()) < self.tolerance # TODO: figure out tolerance

    def getPosition(self) -> degrees:
        """
        Returns the current position of the pivot in degrees (from the encoder)
        """
        return rotationsToDegrees(self.encoder.getPosition())
