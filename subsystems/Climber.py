from commands2 import Subsystem
from wpimath import units
from wpilib import RobotState
from ntcore import NetworkTable, NetworkTableInstance
from rev import SparkMax
from util import FalconLogger


class Climber(Subsystem):
    class Speeds:
        STOP:float = 0
        FORWARD:float = 1
        BACKWARD:float = -1

    # Variable Declaration
    m_speed:float = 0.0
    m_sys_id:int = None
    m_logging:NetworkTable = None

    # Initialization
    def __init__(self, sysId:int) -> None:
        self.m_sys_id = sysId
        self.m_speed = 0.0
        
        self.m_motor:SparkMax = SparkMax(self.m_sys_id, SparkMax.MotorType.kBrushless)

        self.m_gear_ratio = 1
        
        self.m_encoder = self.m_motor.getAbsoluteEncoder()
        

        # consider adding limit switches to motor while running at full speed rather than track rotation?

        self.m_logging = NetworkTableInstance.getDefault().getTable("/Logging/Climber")

    # Periodic Loop
    def periodic(self) -> None:
        # Logging: Write Current Subsystem State
        FalconLogger.logInput("/Climber/Current Position (rotations)", self.getPosition("rotations"))
        FalconLogger.logInput("/Climber/Current Position (degrees)", self.getPosition("degrees"))
        FalconLogger.logInput("/Climber/Current Position (radians)", self.getPosition("radians"))
        FalconLogger.logInput("/Climber/Speed", self.getSpeed())

        # Run Subsystem: Set New State To Subsystem
        if RobotState.isDisabled():
            self.stop()
        else:
            self.run()
        
        # Logging: Write Post Operation Information
        self.m_logging.putNumber( "Climber id (at least in code)", self.m_sys_id )

    # Run the Subsystem
    def run(self) -> None:
        self.m_motor.set(self.m_speed)

    # Stop the Subsystem
    def stop(self) -> None:
        self.m_motor.set(self.Speeds.STOP)

    # Set the Desired Speed
    def setSpeed(self, speed:float) -> None:
        self.m_speed = speed

    # Get the Desired Speed
    def getSpeed(self) -> float:
        return self.m_motor.get()
    
    # Check if Subsystem is at the Desired State
    def atSetpoint(self) -> bool:
        return False

    def getPosition(self, unit:str) -> float:
        """
        returns position in specified type\n
        type = "rotations" returns position in rotations\n
        type = "degrees" returns position in degrees\n
        type = "radians" returns position in radians
        """
        if unit == "rotations":
            return self.m_encoder.getPosition() * self.m_gear_ratio
        elif unit == "degrees":
            return units.rotationsToDegrees(self.m_encoder.getPosition()) * self.m_gear_ratio
        elif unit == "radians":
            return units.rotationsToRadians(self.m_encoder.getPosition()) * self.m_gear_ratio