from commands2 import Subsystem
from wpilib import RobotState, RobotBase, DigitalInput, SmartDashboard
from ntcore import NetworkTable, NetworkTableInstance
from ntcore.util import ntproperty

from rev import SparkMax, SparkMaxConfig, LimitSwitchConfig

from util import FalconLogger, NTTunableFloat

class CoralManipulatorWheel(Subsystem):

    # simHasCoral = lambda a: NTTunableFloat('/SimOutputs/CoralWheel/hasCoral', 0). or ntproprty idk how it works

    class WheelSpeeds:
        STOP: float = 0
        IN: float = -0.6
        SLIGHT_IN: float = -0.05 # only meant for short distances, avoid slamming into mechanism
        OUT: float = 0.6

    # Initialization
    def __init__(self, motor_port:int) -> None:
        # if RobotBase.isSimulation(): self.hasCoral = self.simHasCoral

        # motor
        self.motor = SparkMax( motor_port, SparkMax.MotorType.kBrushless )
        self.ls = self.motor.getReverseLimitSwitch()

        # limit switch
        # self.limit_switch = DigitalInput( 5 ) # BAD

        # config
        motorConfig = SparkMaxConfig()
        motorConfig = motorConfig.setIdleMode( SparkMaxConfig.IdleMode.kBrake )
        motorConfig = motorConfig.inverted(True)

        lsConfig = LimitSwitchConfig()
        lsConfig = lsConfig.reverseLimitSwitchEnabled( False ).reverseLimitSwitchType( LimitSwitchConfig.Type.kNormallyOpen )

        motorConfig.apply(lsConfig)

        self.motor.configure( motorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters )

        # setup
        self.motor_speed:CoralManipulatorWheel.WheelSpeeds = self.WheelSpeeds.STOP

        self.stalled_frames = 0
        # self.free_spin = False
        self.has_coral = False

        SmartDashboard.putData("CoralWheel", self)

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        FalconLogger.logInput('CoralManipulatorWheel/motorOutputCurrent', self.motor.getOutputCurrent())
        FalconLogger.logInput('CoralManipulatorWheel/motorVelocity', self.motor.getEncoder().getVelocity())
        FalconLogger.logInput('CoralManipulatorWheel/limitSwitchState', self.ls.get())

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        self.has_coral = self.ls.get() or self.has_coral
        
        # Logging: Write Post Operation Information
        FalconLogger.logOutput('CoralManipulatorWheel/motorSetSpeed', self.motor_speed)
        FalconLogger.logOutput('CoralManipulatorWheel/hasCoral', self.hasCoral())

    # Run the Subsystem
    def run(self) -> None:
        if self.has_coral and not self.ls.get():
            self.motor.set( self.WheelSpeeds.SLIGHT_IN )
        else:
            self.motor.set( self.motor_speed )


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
        # return self.stalled_frames > 5 or not self.limit_switch.get()
        return self.has_coral

    def set_has_coral(self, val:bool) -> None:
        self.has_coral = val