from commands2 import Subsystem

from wpilib import RobotState, SmartDashboard, RobotBase, Mechanism2d, Color8Bit
from wpimath.controller import PIDController
from wpimath.system.plant import DCMotor

from wpimath.units import radians, rotationsToRadians

from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from rev import SparkMax, SparkMaxSim
# from phoenix6.hardware import CANcoder

import math

from util import FalconLogger

class CoralManipulatorPivot(Subsystem):

    class PivotConstants:
        gear_ratio=25 # or 1/25
        min=0.0
        max=1.0

    # Variable Declaration
    m_sys_id:int = None

    def __init__(self, sysId:int, motor_port:int) -> None:
        self.m_sys_id = sysId

        # using NEO 550
        self.pivotMotor = SparkMax( motor_port, SparkMax.MotorType.kBrushless )

        self.encoder = self.pivotMotor.getAbsoluteEncoder()

        self.controller = PIDController(
            0.0,
            0.0,
            0.0
        )
        self.controller.enableContinuousInput(-math.pi, math.pi)
        SmartDashboard.putData( "/CoralManipulatorPivot/PIDController", self.controller )

        self.desiredPosition: radians = self.getMeasuredPosition()

        self.tolerance = math.pi / 10 #ntproperty("/CoralManipulatorPivot/AtPostionTolerance", math.pi/20, persistent=True)

        ## Simulation Inits
        if RobotBase.isSimulation():
            ## Mech2d
            self.mech = Mechanism2d( 12, 12, Color8Bit(50,50,70) )
            root = self.mech.getRoot( 'CoralPivot', 6, 0 )
            elevator = root.appendLigament('pretendElevator', 6, 90, color=Color8Bit(200,200,170) ) # replace with ref to real elevator sim
            stick = elevator.appendLigament('stick', 3, 90, color=Color8Bit(200,200,170))
            self.pivotArm = stick.appendLigament( 'arm', 6, 180, color=Color8Bit(20,240,20) )
            SmartDashboard.putData( '/CoralManipulatorPivot/mech2d', self.mech)

            ## Motor
            self.simMotor = SparkMaxSim( self.pivotMotor, DCMotor.NEO550() )


    # Periodic Loop
    def periodic(self) -> None:
        # Logging - Write Measured Values
        FalconLogger.logInput('/CoralManipulatorPivot/MeasuredPositionRadians', self.getMeasuredPosition())

        # Run Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging - Write Calculated Values
        FalconLogger.logOutput('/CoralManipulatorPivot/DesiredPosition', self.desiredPosition)
    
    def simulationPeriodic(self):
        self.simMotor.setAppliedOutput()


        self.pivotArm.setAngle( self.pivotArm.getAngle()+1 )

    # Run the Subsystem
    def run(self) -> None:
        val = self.controller.calculate( self.getMeasuredPosition(), self.desiredPosition )
        self.pivotMotor.set( val )

    # Stop the Subsystem
    def stop(self) -> None:
        self.desiredPosition = self.getMeasuredPosition()

    # Set the Desired State Value
    def setSetpoint(self, value:radians) -> None:
        self.desiredPosition = value

    # Get the Desired State Value
    def getSetpoint(self) -> float:
        return self.desiredPosition
    
    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        return abs(self.desiredPosition - self.getMeasuredPosition()) < self.tolerance # figure out tolerance

    def getMeasuredPosition(self) -> radians:
        return rotationsToRadians(self.encoder.getPosition())