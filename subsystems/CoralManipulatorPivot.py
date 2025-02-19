from commands2 import Subsystem

from wpilib import RobotState, SmartDashboard, RobotBase, Mechanism2d, Color8Bit
from wpilib.simulation import SingleJointedArmSim
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor

from wpimath.units import radians, rotationsToRadians, rotationsToDegrees, radiansToDegrees

from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from rev import SparkMax, SparkMaxSim, SparkMaxConfig
from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration

import math
 
from util import FalconLogger

class CoralManipulatorPivot(Subsystem):

    class PivotConstants:
        k_max_velocity=DCMotor.NEO550().freeSpeed * 60
        gear_ratio=1/25 
        min=-math.pi/2 #NOTE: wrong
        max=math.pi/3
        kP=1
        kI=0
        kD=0

    # Variable Declaration
    m_sys_id:int = None

    tolerance = math.pi / 10 #ntproperty("/CoralManipulatorPivot/AtPostionTolerance", math.pi/20, persistent=True)


    def __init__(self, sysId:int, motor_port:int, encoder_port:int) -> None:
        self.m_sys_id = sysId

        ## Motor Init
        # using NEO 550
        self.encoder = CANcoder( encoder_port, 'rio' )
        self.pivotMotor = SparkMax( motor_port, SparkMax.MotorType.kBrushless )
        # config
        motorConfig = SparkMaxConfig()
        motorConfig.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        self.pivotMotor.configure( motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )

        ## Encoder Init
        self.encoder = CANcoder( encoder_port, 'canivore1' )

        self.controller = PIDController(
            0.0,
            0.0,
            0.0
        )
        self.controller.enableContinuousInput(-math.pi, math.pi)
        SmartDashboard.putData( "/CoralManipulatorPivot/PIDController", self.controller )

        self.desiredPosition: radians = self.getMeasuredPosition()

        ## Mech2d
        self.mech = Mechanism2d( 12, 12, Color8Bit(50,50,70) )
        root = self.mech.getRoot( 'CoralPivot', 6, 0 )
        elevator = root.appendLigament('pretendElevator', 6, 90, color=Color8Bit(200,200,170) ) # replace with ref to real elevator mech
        stick = elevator.appendLigament('stick', 3, 90, color=Color8Bit(200,200,170))
        self.pivotArm = stick.appendLigament( 'arm', 6, 180, color=Color8Bit(20,240,20) )
        SmartDashboard.putData( '/CoralManipulatorPivot/mech2d', self.mech)

        ## Simulation Inits
        if RobotBase.isSimulation():
            gearbox = DCMotor.NEO550()
            self.armSim = SingleJointedArmSim(
                gearbox,
                self.PivotConstants.gear_ratio,
                SingleJointedArmSim.estimateMOI( 0.15, 4 ), # NOTE: these are random numbers
                0.15,
                math.pi*3/2,
                math.pi/2,
                True,
                math.pi*3/2,
             )
            self.simMotor = SparkMaxSim( self.pivotMotor, DCMotor.NEO550() )
            # self.simEncoder = self.simMotor.getAbsoluteEncoderSim()
            # self.simEncoder.setPositionConversionFactor(1)


    # Periodic Loop
    def periodic(self) -> None:
        # Logging - Write Measured Values
        FalconLogger.logInput('/CoralManipulatorPivot/MeasuredPositionRadians', self.getMeasuredPosition())

        # Run Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Mech2d
        self.pivotArm.setAngle( radiansToDegrees(self.getMeasuredPosition()) )
        
        # Logging - Write Calculated Values
        FalconLogger.logOutput('/CoralManipulatorPivot/DesiredPosition', self.desiredPosition)
    
    def simulationPeriodic(self):
        
        # self.armSim.setInputVoltage( 12 )

        output = self.controller.calculate( self.getMeasuredPosition(), self.desiredPosition ) * self.PivotConstants.k_max_velocity

        self.armSim.setInput(
            0, output
        )
        self.armSim.update(0.02)

        self.encoder.sim_state.add_position( self.armSim.getAngle() )

        self.pivotArm.setAngle( radiansToDegrees(self.getSetpoint()) )


    def run(self) -> None:
        val = self.controller.calculate( self.getMeasuredPosition(), self.desiredPosition )
        self.pivotMotor.set( val )

    def stop(self) -> None:
        self.desiredPosition = self.getMeasuredPosition()

    def setSetpoint(self, value:radians) -> None:
        self.desiredPosition = value

    def getSetpoint(self) -> radians:
        return self.desiredPosition
    
    def atSetpoint(self) -> bool:
        return abs(self.desiredPosition - self.getMeasuredPosition()) < self.tolerance # figure out tolerance

    def getMeasuredPosition(self) -> radians:
        return rotationsToRadians(self.encoder.get_position().value)