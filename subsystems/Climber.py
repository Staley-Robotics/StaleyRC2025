from commands2 import Subsystem
from wpimath import units, controller
from wpilib import RobotState, Mechanism2d, SmartDashboard, Color, Color8Bit
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
from rev import SparkMax, SparkMaxSim, SparkMaxConfig
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

    class Simulation:
        GEAR_RATIO_SIM:float = 75.6
        ARM_LENGTH_METERS:float = 0.2667
        ARM_MASS_KG:float = 1.35575924
        MIN_ANGLE_RADS:float = 0
        MAX_ANGLE_RADS:float = pi
        IS_SIMULATING_GRAVITY:bool = False
        STARTING_ANGLE_RADS:float = 0
        # note distance per pulse = (angle per revolution) / (pulses per revolution) = (2 * PI rads) / (4096 pulses)
        ENCODER_DIST_PER_PULSE:float = 1


class Climber(Subsystem):

    # Initialization
    def __init__(self, sysId:int) -> None:
        self.__sys_id = sysId
        
        # Init motor and abs encoder
        self.__motor:SparkMax = SparkMax(self.__sys_id, SparkMax.MotorType.kBrushless)
        self.__motor_config:SparkMaxConfig = SparkMaxConfig()
        self.__motor_config.setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake)
        self.__motor.configure(self.__motor_config, self.__motor.ResetMode.kResetSafeParameters, self.__motor.PersistMode.kPersistParameters)
        self.__abs_encoder = self.__motor.getAbsoluteEncoder()

        # Init status vars
        self.__is_climbing = False
        self.__is_un_climbing = False

        # Init sim motor and sim encoders
        self.__motor_sim = SparkMaxSim(self.__motor, DCMotor.NEO(1))
        self.__sim_abs_encoder = self.__motor_sim.getAbsoluteEncoderSim()
        self.__sim_rel_encoder = self.__motor_sim.getRelativeEncoderSim()


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


        self.actual_position = self.getPosition()
        self.desired_position = idk
        self.save_position = self.actual_position
        self.__set_voltage = 0

        # PID setup
        self.kP = 1
        self.kI = 0
        self.kD = 0
        self.kIZone = 0
        self.kTolerance = 0

        self.pid_controller = controller.PIDController(self.kP, self.kI, self.kD)
        self.pid_controller.setIZone(self.kIZone)
        self.pid_controller.setTolerance(self.kTolerance)
        self.pid_controller.enableContinuousInput(0, 360)

        SmartDashboard.putData("PID Controller", self.pid_controller)     
        # self.pid_controller.set
        
        self.mech = Mechanism2d( 3, 3, Color8Bit(Color.kBlue))
        self.root = self.mech.getRoot("root", 1.5, 0)
        self.base = self.root.appendLigament("base", 2, 90, color=Color8Bit(Color.kBrown))
        self.arm = self.base.appendLigament("arm", 1, 90, color=Color8Bit(Color.kGreen))

        SmartDashboard.putData("Mech2d", self.mech)

        # consider adding limit switches to motor while running at full speed rather than track rotation?
        # TODO Ask Tyler what how to limit switch?? Or other programmer like Ben ...

        # self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/Climber")


    # Simulation Periodic Loop
    def simulationPeriodic(self):

        """
        Get voltage from motors (in volts)
        Apply output to sim class (sim class will be a single jointed arm)
        Apply sim output to motor sim object

        """


        output = units.radiansToRotations(self.__motor.getAppliedOutput()) * 12 * 60 # note: self.__motor.getAppliedOutput() outputs in radians per sec
        # note: multiplied by 12 because output needs to be in volts
        # note: multiplied by 60 because conversion from radians per SEC to rotations per MIN


        self.__sim_jointed_arm.setInputVoltage(output) # note: input needs to be in rotations per minute

        self.__sim_jointed_arm.update(0.02)

        sim_velocity = self.__sim_jointed_arm.getVelocity() # note: this returns velocity in rotation per sec
        self.__sim_abs_encoder.iterate(sim_velocity * ClimberConstants.Simulation.GEAR_RATIO_SIM, 0.02)
        self.__sim_rel_encoder.iterate(sim_velocity, 0.02) # TODO add gear ratio (check if gear ratio should be * to or / by sim_velocity) IF NEEDED after testing
        # note: iterate must be done on absolute encoder, relative encoder, and motor

        SmartDashboard.putNumber("sim_motor_output (rotations per minute)", output)
        SmartDashboard.putNumber("sim_velocity", sim_velocity)

        self.arm.setAngle(self.__sim_jointed_arm.getAngleDegrees() - 90)

        SmartDashboard.putNumber("sim_angle", self.__sim_jointed_arm.getAngleDegrees())
        SmartDashboard.putNumber("sim_mech_arm_angle", self.arm.getAngle())



        return super().simulationPeriodic()

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Log Inputs
        FalconLogger.logInput("Climber/MotorInput", self.__motor.get())
        FalconLogger.logInput("Climber/MotorOutput", self.__motor.getAppliedOutput())
        FalconLogger.logInput("Climber/MotorPosition_abs_r", self.__motor.getAbsoluteEncoder().getPosition())
        FalconLogger.logInput("Climber/MotorTemp_c", self.__motor.getMotorTemperature())
        FalconLogger.logInput("Climber/MotorCurrent_a", self.__motor.getOutputCurrent())

        # Update some variables
        self.actual_position = self.getPosition()
        self.save_position = round(self.actual_position) # note: this is used for the command ClimberStay (why is it rounded? idk maybe to stop climber from bakonking itself)

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.safe_stop()
            # self.safe_stop()
        else:
            self.run()
        
                # Logging: Log Outputs
        FalconLogger.logOutput("Climber/TargetPosition", self.desired_position)
        FalconLogger.logOutput("Climber/ActualPosition", self.actual_position) # (up is 180, down is 0) TODO Change this comment after more testing

    # Run the Subsystem
    def run(self) -> None:
        self.__set_voltage = self.pid_controller.calculate(self.getPosition(), self.desired_position )
        self.__motor.setVoltage(self.__set_voltage)

    # Stop the Subsystem
    def force_stop(self) -> None:
        self.__motor.set(ClimberConstants.Speeds.STOP)

    def safe_stop(self) -> None:
        self.setPosition(self.save_position)

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
        return self.pid_controller.atSetpoint()
    
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
        