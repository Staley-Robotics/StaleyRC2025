from commands2 import Subsystem
from wpimath import units, controller
from wpilib import RobotState, Mechanism2d, SmartDashboard, Color, Color8Bit, XboxController
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
        _kP = 5
        _kI = 0
        _kD = 0
        _kFF = 0 # Feed Forward
        _kIZone = 0
        _kTolerance = 1

    class Simulation:
        GEAR_RATIO_SIM:float = 75.6
        ARM_LENGTH_METERS:float = 0.2667
        ARM_MASS_KG:float = 1.35575924
        MIN_ANGLE_RADS:float = -5/8 * pi
        MAX_ANGLE_RADS:float = 5/4 * pi
        IS_SIMULATING_GRAVITY:bool = True
        STARTING_ANGLE_RADS:float = 0
        # note distance per pulse = (angle per revolution) / (pulses per revolution) = (2 * PI rads) / (4096 pulses)
        ENCODER_DIST_PER_PULSE:float = 1


class Climber(Subsystem):

    # Initialization
    def __init__(self, mainId:int, followId:int, encoder_offset) -> None:
        '''
        mainId: main motor ID\n
        followId: following motor ID
        '''

        self.controller = XboxController( 0 )

        self.__main_id = mainId
        self.__follow_id = followId
        ## Init Motors
        self.__main_motor:SparkMax = SparkMax(self.__main_id, SparkMax.MotorType.kBrushless)
        self.__follow_motor:SparkMax = SparkMax(self.__follow_id, SparkMax.MotorType.kBrushless)
        
        # Init motors and config
        self.__motor_config = SparkMaxConfig()
        self.__motor_config = self.__motor_config.setIdleMode(idleMode=SparkMaxConfig.IdleMode.kCoast)
        
        self.__main_motor_closed_loop_config = ClosedLoopConfig()
        self.__main_motor_closed_loop_config = self.__main_motor_closed_loop_config.pidf(
            ClimberConstants.Other._kP,
            ClimberConstants.Other._kI,
            ClimberConstants.Other._kD,
            ClimberConstants.Other._kFF,
            ClosedLoopSlot.kSlot0
        ).setFeedbackSensor(
            ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder
        ).positionWrappingEnabled(False).positionWrappingInputRange(0, 1)

        
        self.__abs_encoder_config = AbsoluteEncoderConfig()
        self.__abs_encoder_config = self.__abs_encoder_config.inverted( True ) # TODO test to see if I need to change to True
        self.__abs_encoder_config = self.__abs_encoder_config.positionConversionFactor(1).velocityConversionFactor(1)
        self.__abs_encoder_config = self.__abs_encoder_config.zeroOffset( encoder_offset )

        # Apply configs
        self.__motor_config.apply(self.__main_motor_closed_loop_config)
        self.__motor_config.apply(self.__abs_encoder_config)
        
        self.__main_motor.configure(self.__motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        self.__motor_config = SparkMaxConfig().follow(self.__main_id, False)
        self.__follow_motor.configure(self.__motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        
        # get motor objects
        self.pid_controller = self.__main_motor.getClosedLoopController()
        self.__abs_encoder = self.__main_motor.getAbsoluteEncoder()

        # Init sim motor and sim encoders
        self.__motor_sim = SparkMaxSim(self.__main_motor, DCMotor.NEO(1))
        self.__sim_abs_encoder = self.__motor_sim.getAbsoluteEncoderSim()
        self.__sim_rel_encoder = self.__motor_sim.getRelativeEncoderSim()

        self.__sim_abs_encoder.setPosition( units.degreesToRotations(-90) )

        self.__sim_jointed_arm = SingleJointedArmSim(
            DCMotor.NEO(2),
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
        # self.save_position = self.getPosition()
        # self.__set_voltage = 0

        # SmartDashboard.putData("/Climber/PID Controller", self.pid_controller)     
        
        self.mech = Mechanism2d( 3, 3, Color8Bit(Color.kBlue))
        self.root = self.mech.getRoot("root", 1.5, 0)
        self.base = self.root.appendLigament("base", 2, 90, color=Color8Bit(Color.kBrown))
        self.arm = self.base.appendLigament("arm", 1, 90, color=Color8Bit(Color.kGreen))
        self.armSim = self.base.appendLigament("armSim", 4, 90, color=Color8Bit(Color.kRed))
        self.armSetPoint = self.base.appendLigament("armSetPoint", 1, 90, color=Color8Bit(Color.kYellow), lineWidth=3)

        SmartDashboard.putData("/Climber/mech2d", self.mech)
        SmartDashboard.putData("/Climber", self)


    # Simulation Periodic Loop
    def simulationPeriodic(self):

        """
        Get voltage from motors (in volts)
        Apply output to sim class (sim class will be a single jointed arm)
        Apply sim output to motor sim object

        """
        
        self.__motor_sim.setMotorCurrent(0)

        sim_velocity = self.__sim_jointed_arm.getVelocity() # note: this returns velocity in radians per sec
        self.__motor_sim.iterate( units.radiansToRotations( sim_velocity ) * 60, 12.0, 0.02 )
        # self.__sim_abs_encoder.iterate(sim_velocity * ClimberConstants.Simulation.GEAR_RATIO_SIM, 0.02)
        self.__sim_rel_encoder.iterate( units.radiansToRotations(sim_velocity) * 60 * 1/ClimberConstants.Other.GEAR_RATIO, 0.02) # TODO add gear ratio (check if gear ratio should be * to or / by sim_velocity) IF NEEDED after testing
        # note: iterate must be done on absolute encoder, relative encoder, and motor

        # SmartDashboard.putNumber("/Climber/sim_outputs/sim_motor_output (rotations per minute)", output)
        SmartDashboard.putNumber("/Climber/sim_outputs/sim_velocity", sim_velocity)

        self.armSim.setAngle(self.__sim_jointed_arm.getAngleDegrees() - 90)

        SmartDashboard.putNumber("/Climber/sim_outputs/sim_angle", self.__sim_jointed_arm.getAngleDegrees())
        # SmartDashboard.putNumber("/Climber/sim_outputs/sim_mech_arm_angle", self.arm.getAngle() + 90)

        output = self.__motor_sim.getAppliedOutput() * 12  # note: self.__motor.getAppliedOutput() outputs in radians per sec
        # note: multiplied by 12 because output needs to be in volts


        self.__sim_jointed_arm.setInputVoltage(output) # note: input needs to be in rotations per minute

        self.__sim_jointed_arm.update(0.02)


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
        # self.save_position = round(self.getPosition()) # note: this is used for the command ClimberStay (why is it rounded? idk maybe to stop climber from bakonking itself)
        self.arm.setAngle(self.getPosition() - 90)
        self.armSetPoint.setAngle(self.desired_position - 90)

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
        # self.__main_motor.set(self.controller.getLeftY())
        # return
        self.pid_controller.setReference(
            units.degreesToRotations(self.desired_position),
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
            )

    # Stop the Subsystem
    def force_stop(self) -> None:
        # self.__main_motor.set(ClimberConstants.Speeds.STOP)
        pass

    def safe_stop(self) -> None:
        self.setPosition(self.getPosition())

    # Set the Desired Position
    def setPosition(self, position:units.degrees):
        """Sets the desired position of the climber
    
        :param position: the desired position in degrees
        """
        self.desired_position = position

        # self.pid_controller.setReference(
        #     units.degreesToRotations(self.desired_position),
        #     SparkBase.ControlType.kPosition,
        #     ClosedLoopSlot.kSlot0
        #     )

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
        val = units.rotationsToDegrees(self.__abs_encoder.getPosition())

        if val > 270:
            val -= 360
        elif val < -90:
            val += 360
            
        return val
        