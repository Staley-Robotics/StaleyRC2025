from commands2 import Subsystem
from wpimath import units, controller
from wpilib import RobotState, RobotBase, Mechanism2d, SmartDashboard, Encoder, Color, Color8Bit
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
# from ntcore import NetworkTable, NetworkTableInstance
from rev import SparkMax, SparkMaxSim
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
        GEAR_RATIO_SIM:float = 75.6 # might need to use this
        ARM_LENGTH_METERS:float = 0.2667
        ARM_MASS_KG:float = 1.35575924
        MIN_ANGLE_RADS:float = 0
        MAX_ANGLE_RADS:float = pi
        IS_SIMULATING_GRAVITY:bool = False
        STARTING_ANGLE_RADS:float = 0
        # distance per pulse = (angle per revolution) / (pulses per revolution)
        #  = (2 * PI rads) / (4096 pulses) MAYBE??
        ENCODER_DIST_PER_PULSE:float = 1


class Climber(Subsystem):


    # Variable Declaration
    # m_speed:float = 0.0
    # m_sys_id:int = None
    # m_logging:NetworkTable = None

    # Initialization
    def __init__(self, sysId:int) -> None:
        self.__sys_id = sysId

        # self.m_speed = 0.0
        
        # Init motor and abs encoder
        self.__motor:SparkMax = SparkMax(self.__sys_id, SparkMax.MotorType.kBrushless)
        self.__abs_encoder = self.__motor.getAbsoluteEncoder()
        # self.__rel_encoder = self.__motor.getEncoder()

        # self.__encoder_test = self.

        # Init sim motor and sim encoders
        self.__motor_sim = SparkMaxSim(self.__motor, DCMotor.NEO(1))
        self.__sim_abs_encoder = self.__motor_sim.getAbsoluteEncoderSim()
        self.__sim_rel_encoder = self.__motor_sim.getRelativeEncoderSim()


        self.__sim_jointed_arm = SingleJointedArmSim(
            DCMotor.NEO(1),
            ClimberConstants.Simulation.GEAR_RATIO_SIM, # might change to ClimberConstants.Simulation.GEAR_RATIO_SIM
            SingleJointedArmSim.estimateMOI(ClimberConstants.Simulation.ARM_LENGTH_METERS, ClimberConstants.Simulation.ARM_MASS_KG),
            ClimberConstants.Simulation.ARM_LENGTH_METERS,
            ClimberConstants.Simulation.MIN_ANGLE_RADS,
            ClimberConstants.Simulation.MAX_ANGLE_RADS,
            ClimberConstants.Simulation.IS_SIMULATING_GRAVITY,
            ClimberConstants.Simulation.STARTING_ANGLE_RADS,
        )


        self.actual_position = self.__abs_encoder.getPosition()
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
        Apply sim output to to motor sim object

        """


        output = units.radiansToRotations(self.__motor.getAppliedOutput()) * 12 * 60 # self.__motor.getAppliedOutput() outputs in radians per sec
        # multiplied by 12 because output needs to be volts (or something like that)
        # multiplied by 60 because conversion from radians per SEC to rotations per MIN


        self.__sim_jointed_arm.setInputVoltage(output) # input needs to be in rotations per minute

        self.__sim_jointed_arm.update(0.02)

        sim_velocity = self.__sim_jointed_arm.getVelocity() # this returns velocity in rotation per sec <-- dunno if this is a problem
        self.__motor_sim.iterate(sim_velocity, 12, 0.02)
        self.__sim_abs_encoder.iterate(sim_velocity * ClimberConstants.Simulation.GEAR_RATIO_SIM, 0.02) # TODO add gear ratio (check if gear ratio should be * or /)
        self.__sim_rel_encoder.iterate(sim_velocity, 0.02) # TODO add gear ratio (check if gear ratio should be * or /)
        #iterate must be done on absolute encoder, relative encoder, and motor
        #* ClimberConstants.Simulation.GEAR_RATIO_SIM

        # print(f"output: {output},  sim_velocity: {sim_velocity}")
        SmartDashboard.putNumber("sim_motor_output (rotations per minute)", output)
        SmartDashboard.putNumber("sim_velocity", sim_velocity)

        self.arm.setAngle(self.__sim_jointed_arm.getAngleDegrees() - 90)

        SmartDashboard.putNumber("sim_angle", self.__sim_jointed_arm.getAngleDegrees())
        SmartDashboard.putNumber("sim_mech_arm_angle", self.arm.getAngle())



        return super().simulationPeriodic()

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Log Inputs
        FalconLogger.logInput("/Climber/Current Position (degrees)", self.getPosition())
        FalconLogger.logInput("/Climber/Speed (degrees per sec)", self.getSpeed())
        FalconLogger.logInput("/Climber/Motor Voltage from motor", self.__motor.getBusVoltage() * self.__motor.getAppliedOutput())
        #apparently multiplying 

        # Logging: Log Outputs
        FalconLogger.logOutput("/Climber/Motor Voltage from PID calculation", self.__set_voltage)
        FalconLogger.logOutput("/Climber/Desired Position (degrees)", self.desired_position) # (up is 180, down is 0) TODO change key for this input after testing


        # Update some variables
        self.actual_position = self.__abs_encoder.getPosition()
        self.save_position = round(self.actual_position) # this is used for the command ClimberStay (why is it rounded? idk maybe to stop climber from bakonking itself)
        # self.desired_position = self.pid_controller.getSetpoint()

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        # self.m_logging.putNumber( "Climber id (at least in code)", self.m_sys_id )

    # Run the Subsystem
    def run(self) -> None:
        self.__set_voltage = self.pid_controller.calculate(self.getPosition("degrees"), self.desired_position )
        self.__motor.setVoltage(self.__set_voltage)

    # Stop the Subsystem
    def stop(self) -> None:
        self.__motor.set(ClimberConstants.Speeds.STOP)

    # Set the Desired Position
    def setPosition(self, position:units.degrees):
        """Sets the desired position of the climber
    
        :param position: the desired position in degrees
        """
        self.desired_position = position
        # self.pid_controller.setSetpoint(position)

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