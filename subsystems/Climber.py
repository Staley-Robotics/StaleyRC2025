from commands2 import Subsystem
from wpimath import units, controller
from wpilib import RobotState, Mechanism2d, SmartDashboard, Color, Color8Bit
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
from rev import SparkBase, SparkMax, SparkMaxSim, SparkMaxConfig, AbsoluteEncoderConfig, ClosedLoopSlot, ClosedLoopConfig
from util import FalconLogger
from math import pi


idk = 0 #tbd :D

class ClimberConstants():
    class Speeds:
        STOP:float = 0
        FORWARD:float = 1
        BACKWARD:float = -1

    class Other:
        GEAR_RATIO:float = 1/75.6
        _kP = 1
        _kI = 0
        _kD = 0
        _kFF = 0 # Feed Forward
        _kIZone = 0
        _kTolerance = 1

    class Simulation:
        GEAR_RATIO_SIM:float = 75.6
        ARM_LENGTH_METERS:float = 0.2667
        ARM_MASS_KG:float = 1.35575924
        MIN_ANGLE_RADS:float = 0
        MAX_ANGLE_RADS:float = pi
        IS_SIMULATING_GRAVITY:bool = False
        STARTING_ANGLE_RADS:float = 1
        # note distance per pulse = (angle per revolution) / (pulses per revolution) = (2 * PI rads) / (4096 pulses)
        ENCODER_DIST_PER_PULSE:float = 1


class Climber(Subsystem):

    # Initialization
    def __init__(self, mainId:int, followId:int, encoder_offset) -> None:
        '''
        mainId: main motor ID\n
        followId: following motor ID
        '''
        self.__main_id = mainId
        self.__follow_id = followId
        
        # Init motors and config
        # PID setup
        self.__main_motor_closed_loop_config = ClosedLoopConfig()
        self.__main_motor_closed_loop_config = self.__main_motor_closed_loop_config.pidf(
            ClimberConstants.Other._kP,
            ClimberConstants.Other._kI,
            ClimberConstants.Other._kD,
            ClimberConstants.Other._kFF,
            ClosedLoopSlot.kSlot0
        )

        # Init Motors
        self.__main_motor:SparkMax = SparkMax(self.__main_id, SparkMax.MotorType.kBrushless)
        self.__follow_motor:SparkMax = SparkMax(self.__follow_id, SparkMax.MotorType.kBrushless)

        # Make Config
        self.__motor_config:SparkMaxConfig = SparkMaxConfig()
        self.__motor_config = self.__motor_config.setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake)
        
        
        # Init encoder and config
        self.__abs_encoder_config = AbsoluteEncoderConfig()
        self.__abs_encoder_config = self.__abs_encoder_config.setSparkMaxDataPortConfig()
        self.__abs_encoder_config = self.__abs_encoder_config.inverted( False ) # TODO test to see if I need to change to True
        self.__abs_encoder_config = self.__abs_encoder_config.positionConversionFactor(1).velocityConversionFactor(1)
        self.__abs_encoder_config = self.__abs_encoder_config.zeroOffset( encoder_offset )

        
        self.__abs_encoder = self.__main_motor.getAbsoluteEncoder()

        # Apply configs
        self.__motor_config.apply(self.__main_motor_closed_loop_config)
        self.__motor_config.apply(self.__abs_encoder_config)
        
        self.__main_motor.configure(self.__motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        self.__motor_config = self.__motor_config.follow(self.__main_id, False)
        self.__follow_motor.configure(self.__motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.pid_controller = self.__main_motor.getClosedLoopController()


        # Init sim motor and sim encoders
        self.__motor_sim = SparkMaxSim(self.__main_motor, DCMotor.NEO(1))
        self.__sim_abs_encoder = self.__motor_sim.getAbsoluteEncoderSim()
        self.__sim_rel_encoder = self.__motor_sim.getRelativeEncoderSim()

        self.__sim_abs_encoder.setPosition( units.degreesToRotations(-90) )

        self.__sim_jointed_arm = SingleJointedArmSim(
            DCMotor.NEO(1),
            ClimberConstants.Simulation.GEAR_RATIO_SIM,
            SingleJointedArmSim.estimateMOI(ClimberConstants.Simulation.ARM_LENGTH_METERS, ClimberConstants.Simulation.ARM_MASS_KG),
            ClimberConstants.Simulation.ARM_LENGTH_METERS,
            ClimberConstants.Simulation.MIN_ANGLE_RADS,
            ClimberConstants.Simulation.MAX_ANGLE_RADS,
            ClimberConstants.Simulation.IS_SIMULATING_GRAVITY,
            ClimberConstants.Simulation.STARTING_ANGLE_RADS,
        )


        # self.actual_position = self.getPosition()
        self.desired_position = self.getPosition()
        self.save_position = self.getPosition()
        self.__set_voltage = 0

        # PID setup
        __mainMotorClosedLoopConfig = ClosedLoopConfig()
        __mainMotorClosedLoopConfig = __mainMotorClosedLoopConfig.pidf(
            ClimberConstants.Other._kP,
            ClimberConstants.Other._kI,
            ClimberConstants.Other._kD,
            ClimberConstants.Other._kFF,
            ClosedLoopSlot.kSlot0
        )

        # SmartDashboard.putData("/Climber/PID Controller", self.pid_controller)     
        
        self.mech = Mechanism2d( 3, 3, Color8Bit(Color.kBlue))
        self.root = self.mech.getRoot("root", 1.5, 0)
        self.base = self.root.appendLigament("base", 2, 90, color=Color8Bit(Color.kBrown))
        self.arm = self.base.appendLigament("arm", 1, 90, color=Color8Bit(Color.kGreen))

        SmartDashboard.putData("/Climber/mech2d", self.mech)
        SmartDashboard.putData("/Climber", self)


    # Simulation Periodic Loop
    def simulationPeriodic(self):

        """
        Get voltage from motors (in volts)
        Apply output to sim class (sim class will be a single jointed arm)
        Apply sim output to motor sim object

        """


        output = self.__motor_sim.getAppliedOutput() * 12  # note: self.__motor.getAppliedOutput() outputs in radians per sec
        # note: multiplied by 12 because output needs to be in volts


        self.__sim_jointed_arm.setInputVoltage(output) # note: input needs to be in rotations per minute

        self.__sim_jointed_arm.update(0.02)

        sim_velocity = self.__sim_jointed_arm.getVelocity() # note: this returns velocity in rotation per sec
        self.__sim_abs_encoder.iterate(sim_velocity * ClimberConstants.Simulation.GEAR_RATIO_SIM, 0.02)
        self.__sim_rel_encoder.iterate(sim_velocity, 0.02) # TODO add gear ratio (check if gear ratio should be * to or / by sim_velocity) IF NEEDED after testing
        # note: iterate must be done on absolute encoder, relative encoder, and motor

        SmartDashboard.putNumber("/Climber/sim_outputs/sim_motor_output (rotations per minute)", output)
        SmartDashboard.putNumber("/Climber/sim_outputs/sim_velocity", sim_velocity)

        self.arm.setAngle(self.__sim_jointed_arm.getAngleDegrees() - 90)

        SmartDashboard.putNumber("/Climber/sim_outputs/sim_angle", self.__sim_jointed_arm.getAngleDegrees())
        SmartDashboard.putNumber("/Climber/sim_outputs/sim_mech_arm_angle", self.arm.getAngle())



        return super().simulationPeriodic()

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Log Inputs
        FalconLogger.logInput("Climber/MotorInput", self.__main_motor.get())
        FalconLogger.logInput("Climber/MotorOutput", self.__main_motor.getAppliedOutput())
        FalconLogger.logInput("Climber/MotorPosition_abs_r", self.__abs_encoder.getPosition())
        FalconLogger.logInput("Climber/MotorTemp_c", self.__main_motor.getMotorTemperature())
        FalconLogger.logInput("Climber/MotorCurrent_a", self.__main_motor.getOutputCurrent())

        # Update some variables
        self.save_position = round(self.getPosition()) # note: this is used for the command ClimberStay (why is it rounded? idk maybe to stop climber from bakonking itself)

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.safe_stop()
            # self.safe_stop()
        else:
            self.run()
        
                # Logging: Log Outputs
        FalconLogger.logOutput("Climber/TargetPosition", self.desired_position)
        FalconLogger.logOutput("Climber/ActualPosition", self.getPosition()) # (up is 180, down is 0) TODO Change this comment after more testing
        # FalconLogger.logOutput("Climber/PID Calculation (voltage)", self.__set_voltage)

    # Run the Subsystem
    def run(self) -> None:
        self.pid_controller.setReference(
            units.degreesToRotations(self.desired_position),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
            )

    # Stop the Subsystem
    def force_stop(self) -> None:
        self.__main_motor.set(ClimberConstants.Speeds.STOP)

    def safe_stop(self) -> None:
        self.setPosition(self.getPosition())

    # Set the Desired Position
    def setPosition(self, position:units.degrees):
        """Sets the desired position of the climber
    
        :param position: the desired position in degrees
        """
        self.desired_position = position

    # Get Desired Position
    def getDesiredPosition(self):
        """Returns desired position of the climber in degrees (not climber's motor, the actual climber)"""
        return self.desired_position
    
    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        """Returns true if climber is at desired position (not climber's motor, the actual climber)"""
        return abs(self.getPosition() - self.desired_position) < ClimberConstants.Other._kTolerance
    
    # Get speed
    def getSpeed(self):
        """Returns speed of climber arm in degrees per second"""
        return units.rotationsToDegrees(self.__abs_encoder.getVelocity() * ClimberConstants.Other.GEAR_RATIO)


    def getPosition(self) -> float:
        """
        Gets position of clmiber in degrees (not climber's motor, the actual climber)

        :returns: position of climber in degrees
        """
        return units.rotationsToDegrees(self.__abs_encoder.getPosition()) * ClimberConstants.Other.GEAR_RATIO
        