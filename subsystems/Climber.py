from commands2 import Subsystem
from wpimath import units, controller
from wpilib import RobotState, RobotBase, Mechanism2d, SmartDashboard
# from ntcore import NetworkTable, NetworkTableInstance
from rev import SparkMax
from util import FalconLogger


idk = 0 #tbd :D

class ClimberConstants():
    class Speeds:
        STOP:float = 0
        FORWARD:float = 1
        BACKWARD:float = -1

    class Other:
        GEAR_RATIO:float = 1/75.6


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
        


        self.actual_position = self.__encoder.getPosition()
        self.desired_position = idk

        # PID setup
        self.kP = 0
        self.kI = 0
        self.kD = 0
        self.kIZone = 0
        self.kTolerance = 0

        self.pid_controller = controller.PIDController(self.kP, self.kI, self.kD)
        self.pid_controller.setIZone(self.kIZone)
        self.pid_controller.setTolerance(self.kTolerance)      
        # self.pid_controller.set



        if RobotBase.isSimulation():
            self.mech = Mechanism2d( 3, 3 )
            self.root = self.mech.getRoot("root", 0, 0)
            self.base = self.root.appendLigament("base", 2, 0)
            self.arm = self.base.appendLigament("arm", 2, 90)

        # consider adding limit switches to motor while running at full speed rather than track rotation?
        # TODO Ask Tyler what how to limit switch?? Or other programmer like Ben ...

        # self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/Climber")


    # Simulation Periodic Loop
    def simulationPeriodic(self):
        return super().simulationPeriodic()

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        FalconLogger.logInput("/Climber/Current Position (rotations)", self.getPosition("rotations"))
        FalconLogger.logInput("/Climber/Current Position (degrees)", self.getPosition("degrees"))
        FalconLogger.logInput("/Climber/Current Position (radians)", self.getPosition("radians"))
        FalconLogger.logInput("/Climber/Speed", self.getSpeed())

        SmartDashboard.putData("desired position", self.desired_position)

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
        return self.__encoder.getVelocity * ClimberConstants.Other.GEAR_RATIO


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