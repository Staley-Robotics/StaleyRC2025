from commands2 import Subsystem
from wpimath import units, controller
from wpilib import RobotState, RobotBase, Mechanism2d, SmartDashboard, Encoder
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
        IS_SIMULATING_GRAVITY:bool = True
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
        
        # Simple (maybe) stuff
        self.__motor:SparkMax = SparkMax(self.__sys_id, SparkMax.MotorType.kBrushless)
        self.__encoder = self.__motor.getAbsoluteEncoder()

        # self.__encoder_test = self.

        self.__motor_sim = SparkMaxSim(self.__motor, DCMotor.NEO(1))
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

        


        self.actual_position = self.__encoder.getPosition()
        self.desired_position = idk

        # PID setup
        self.kP = 1
        self.kI = 0
        self.kD = 0
        self.kIZone = 0
        self.kTolerance = 0

        self.pid_controller = controller.PIDController(self.kP, self.kI, self.kD)
        self.pid_controller.setIZone(self.kIZone)
        self.pid_controller.setTolerance(self.kTolerance) 

        SmartDashboard.putData("PID Controller", self.pid_controller)     
        # self.pid_controller.set

        

        
        self.mech = Mechanism2d( 3, 3 )
        self.root = self.mech.getRoot("root", 0, 0)
        self.base = self.root.appendLigament("base", 2, 0)
        self.arm = self.base.appendLigament("arm", 2, 90)

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


        output = self.__motor.getAppliedOutput() * 12
        self.__sim_jointed_arm.setInputVoltage(output)

        self.__sim_jointed_arm.update(0.02)

        sim_velocity = self.__sim_jointed_arm.getVelocity()
        self.__motor_sim.iterate(sim_velocity, 12, 0.02)

        # print(f"output: {output},  sim_velocity: {sim_velocity}")
        SmartDashboard.putNumber("sim_motor_output", output)
        SmartDashboard.putNumber("sim_velocity", sim_velocity)



        return super().simulationPeriodic()

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        FalconLogger.logInput("/Climber/Current Position (rotations)", self.getPosition("rotations"))
        FalconLogger.logInput("/Climber/Current Position (degrees)", self.getPosition("degrees"))
        FalconLogger.logInput("/Climber/Current Position (radians)", self.getPosition("radians"))
        FalconLogger.logInput("/Climber/Speed", self.getSpeed())

        SmartDashboard.putNumber("desired position", self.desired_position)

        # Update some variables
        self.actual_position = self.__encoder.getPosition()
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
        self.__motor.setVoltage(self.pid_controller.calculate(self.getPosition("degrees"), self.desired_position ))

    # Stop the Subsystem
    def stop(self) -> None:
        self.__motor.set(ClimberConstants.Speeds.STOP)

    # Set the Desired Position
    def setPosition(self, position:float):
        """Sets the desired position of the climber
    
        :param position: the desired position in degrees
        """
        self.desired_position = position
        # self.pid_controller.setSetpoint(position)

    # Get Desired Position
    def getDesiredPosition(self):
        """Self explanatory"""
        return self.desired_position
    
    # Get Actual Position
    def getActualPosition(self):
        """Self explanatory"""
        return self.actual_position
    
    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        """Returns true if climber is at desired position"""
        return self.pid_controller.atSetpoint()
    
    # Get speed
    def getSpeed(self):
        """Returns speed of climber arm in rotations per second"""
        return self.__encoder.getVelocity() * ClimberConstants.Other.GEAR_RATIO


    def getPosition(self, unit:str) -> float:
        """
        returns position in specified type\n
        type = "rotations" returns position in rotations\n
        type = "degrees" returns position in degrees\n
        type = "radians" returns position in radians
        """
        if unit == "rotations":
            return self.__encoder.getPosition() * ClimberConstants.Other.GEAR_RATIO
        elif unit == "degrees":
            return units.rotationsToDegrees(self.__encoder.getPosition()) * ClimberConstants.Other.GEAR_RATIO
        elif unit == "radians":
            return units.rotationsToRadians(self.__encoder.getPosition()) * ClimberConstants.Other.GEAR_RATIO