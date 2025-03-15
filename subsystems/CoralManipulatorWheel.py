from commands2 import Subsystem
from wpilib import RobotState, DigitalInput
from ntcore import NetworkTable, NetworkTableInstance

from rev import SparkMax, SparkMaxConfig, LimitSwitchConfig

from util import FalconLogger

class CoralManipulatorWheel(Subsystem):

    class WheelSpeeds:
        STOP = 0
        IN = -1
        OUT = 1

    # Initialization
    def __init__(self, motor_port:int) -> None:
        # motor
        self.motor = SparkMax( motor_port, SparkMax.MotorType.kBrushless )
        self.rlSwitch = self.motor.getReverseLimitSwitch()

        # limit switch
        self.limit_switch = DigitalInput( 5 )

        # config
        motorConfig = SparkMaxConfig()
        motorConfig.setIdleMode( SparkMaxConfig.IdleMode.kBrake )

        ls_cfg = LimitSwitchConfig()
        ls_cfg = ls_cfg.reverseLimitSwitchType( LimitSwitchConfig.Type.kNormallyOpen )
        ls_cfg = ls_cfg.reverseLimitSwitchEnabled( True )

        motorConfig.apply( ls_cfg )

        self.motor.configure( motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )

        # setup
        self.motor_speed:CoralManipulatorWheel.WheelSpeeds = self.WheelSpeeds.STOP

        self.stalled_frames = 0
        self.free_spin = False

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        FalconLogger.logInput('CoralManipulatorWheel/motorOutputCurrent', self.motor.getOutputCurrent())
        FalconLogger.logInput('CoralManipulatorWheel/motorVelocity', self.motor.getEncoder().getVelocity())

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput('CoralManipulatorWheel/motorSetSpeed', self.motor_speed)
        FalconLogger.logOutput('CoralManipulatorWheel/hasCoral', self.hasCoral())

    # Run the Subsystem
    def run(self) -> None:
        #self.motor.configureAsync()
        pass

    # Stop the Subsystem
    def stop(self) -> None:
        self.motor_speed = self.WheelSpeeds.STOP

    # Set the Desired State Value
    def setSpeed(self, speed:float) -> None:
        """
        :param speed: percent speed the motor should run at
        """
        self.motor_speed = speed

    # Get the Desired State Value
    def getSetSpeed(self) -> float:
        return self.motor_speed
    
    def hasCoral(self) -> bool:
        #return self.stalled_frames > 5 or not self.limit_switch.get()
        return self.rlSwitch.get()