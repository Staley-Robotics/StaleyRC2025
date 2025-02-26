from commands2 import Subsystem
from wpilib import RobotState
from ntcore import NetworkTable, NetworkTableInstance

from rev import SparkMax, SparkMaxConfig

from util import FalconLogger

class CoralManipulatorWheel(Subsystem):

    class WheelSpeeds:
        STOP = 0
        IN = -1
        OUT = 1

    # Variable Declaration
    m_sys_id:int = None

    # Initialization
    def __init__(self, sysId:int, motor_port:int) -> None:
        self.m_sys_id = sysId

        # motor
        self.motor = SparkMax( motor_port, SparkMax.MotorType.kBrushless )

        # config
        motorConfig = SparkMaxConfig()
        motorConfig.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        self.motor.configure( motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )

        # setup
        self.motor_speed:CoralManipulatorWheel.WheelSpeeds = self.WheelSpeeds.STOP

        self.stalled_frames = 0
        self.free_spin = False

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        FalconLogger.logInput('/CoralManipulatorWheel/motorOutputCurrent', self.motor.getOutputCurrent())
        FalconLogger.logInput('/CoralManipulatorWheel/motorVelocity', self.motor.getEncoder().getVelocity())

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput('/CoralManipulatorWheel/motorSetSpeed', self.motor_speed)
        FalconLogger.logOutput('/CoralManipulatorWheel/hasCoral', self.hasCoral())

    # Run the Subsystem
    def run(self) -> None:
        self.motor.set( self.motor_speed )

        # output current / desired speed should account for only checking this when importing the coral
        # the velocity check should account for the motor not actually moving
        ##NOTE: need to run coral through the prototype to try and get some numbers
        # if self.motor.getOutputCurrent() > 0 and abs(self.encoder.getVelocity()) < 0.5: 
        #     self.stalled_frames += 1
        # else:
        #     self.stalled_frames = 0

    # Stop the Subsystem
    def stop(self) -> None:
        self.motor_speed = self.WheelSpeeds.STOP

    # Set the Desired State Value
    def setSpeed(self, speed:float) -> None:
        '''
        :param speed: percent speed the motor should run at
        '''
        self.motor_speed = speed

    # Get the Desired State Value
    def getSetSpeed(self) -> float:
        return self.motor_speed
    
    def hasCoral(self) -> bool:
        return self.stalled_frames > 5