from enum import Enum, auto
from typing import Callable

from commands2 import Subsystem
from ntcore import NetworkTable, NetworkTableInstance
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Rectangle2d, Translation2d

class RobotMode(Enum):
    TEST = auto()
    COMPETITION = auto()
    DEMO = auto()
    NONE = auto()

class ReefspaceField(Enum):
    class kBlue(Enum):
        LEFT_FIELD  = Rectangle2d( Translation2d(16, 9), Translation2d(-1,4.25 ) )
        RIGHT_FIELD = Rectangle2d( Translation2d(16, 4.25), Translation2d( -1, -1 ) )
        LEFT_HUMAN  = Rectangle2d( Translation2d(), Translation2d() )
        RIGHT_HUMAN = Rectangle2d( Translation2d(), Translation2d() )
        PROCESSOR   = Rectangle2d( Translation2d(), Translation2d() )
        BARGE       = Rectangle2d( Translation2d(), Translation2d() )
        
        REEF_TOPLEFT      = Rectangle2d( Translation2d(), Translation2d() )
        REEF_TOPCENTER    = Rectangle2d( Translation2d(), Translation2d() )
        REEF_TOPRIGHT     = Rectangle2d( Translation2d(), Translation2d() )
        REEF_BOTTOMLEFT   = Rectangle2d( Translation2d(), Translation2d() )
        REEF_BOTTOMCENTER = Rectangle2d( Translation2d(), Translation2d() )
        REEF_BOTTOMRIGHT  = Rectangle2d( Translation2d(), Translation2d() )
    
    class kRed(Enum):
        LEFT_FIELD  = Rectangle2d( Translation2d(-1, -1 ), Translation2d(16, 4.25 ) )
        RIGHT_FIELD = Rectangle2d( Translation2d(-1, 4.25), Translation2d( 16, 9 ) )
        LEFT_HUMAN  = Rectangle2d( Translation2d(), Translation2d() )
        RIGHT_HUMAN = Rectangle2d( Translation2d(), Translation2d() )
        PROCESSOR   = Rectangle2d( Translation2d(), Translation2d() )
        BARGE       = Rectangle2d( Translation2d(), Translation2d() )
        
        REEF_TOPLEFT      = Rectangle2d( Translation2d(), Translation2d() )
        REEF_TOPCENTER    = Rectangle2d( Translation2d(), Translation2d() )
        REEF_TOPRIGHT     = Rectangle2d( Translation2d(), Translation2d() )
        REEF_BOTTOMLEFT   = Rectangle2d( Translation2d(), Translation2d() )
        REEF_BOTTOMCENTER = Rectangle2d( Translation2d(), Translation2d() )
        REEF_BOTTOMRIGHT  = Rectangle2d( Translation2d(), Translation2d() )

class ReefscapePositions(Enum):
    Processor = Pose2d( 0, 0, Rotation2d( 90.0 ) )

    class Barge(Enum):
        LEFT = Pose2d( 0, 0, Rotation2d( 0 ) )
        MIDDLE = Pose2d( 0, 0, Rotation2d( 0 ) )
        RIGHT = Pose2d( 0, 0, Rotation2d( 0 ) )
    
    class Reef(Enum):
        R1  = Pose2d( 4.998, 5.199, Rotation2d.fromDegrees( -120.0 ) )
        R2  = Pose2d( 5.263, 5.053, Rotation2d.fromDegrees( -120.0 ) )
        R3  = Pose2d( 5.758, 4.194, Rotation2d.fromDegrees(  180.0 ) )
        R4  = Pose2d( 4.763, 3.852, Rotation2d.fromDegrees(  180.0 ) )
        R5  = Pose2d( 5.272, 3.000, Rotation2d.fromDegrees(  120.0 ) )
        R6  = Pose2d( 4.998, 2.830, Rotation2d.fromDegrees(  120.0 ) )
        R7  = Pose2d( 4.000, 2.829, Rotation2d.fromDegrees(   60.0 ) )
        R8  = Pose2d( 3.710, 3.000, Rotation2d.fromDegrees(   60.0 ) )
        R9  = Pose2d( 3.200, 3.860, Rotation2d.fromDegrees(    0.0 ) )
        R10 = Pose2d( 3.200, 4.187, Rotation2d.fromDegrees(    0.0 ) )
        R11 = Pose2d( 3.716, 5.058, Rotation2d.fromDegrees(  -60.0 ) )
        R12 = Pose2d( 3.976, 5.212, Rotation2d.fromDegrees(  -60.0 ) )

    class Source(Enum):
        class Left(Enum):
            INNER  = Pose2d( 0.660, 6.681, Rotation2d.fromDegrees( -27.0 ) )
            MIDDLE = Pose2d( 1.200, 7.051, Rotation2d.fromDegrees( -27.0 ) )
            OUTER  = Pose2d( 1.700, 7.413, Rotation2d.fromDegrees( -27.0 ) )

        class Right(Enum):
            INNER  = Pose2d( 0.734, 1.340, Rotation2d.fromDegrees( 46.5 ) )
            MIDDLE = Pose2d( 1.202, 1.029, Rotation2d.fromDegrees( 46.5 ) )
            OUTER  = Pose2d( 1.459, 0.790, Rotation2d.fromDegrees( 46.5 ) )


class RobotRegion(Enum):
    NONE = auto()
    ALGAE = auto()
    BARGE = auto()
    REEF = auto()
    SOURCE = auto()

class SourceSide(Enum):
    LEFT = auto()
    RIGHT = auto()
    AUTO = auto()

class SourceSelect(Enum):
    INNER = auto()
    MIDDLE = auto()
    OUTER = auto()


class ReefTarget(Enum):
    L1 = auto()
    L2 = auto()
    L3 = auto()
    L4 = auto()


class ReefSide(Enum):
    R1 = auto()
    R2 = auto()
    R3 = auto()
    R4 = auto()
    R5 = auto()
    R6 = auto()
    R7 = auto()
    R8 = auto()
    R9 = auto()
    R10 = auto()
    R11 = auto()
    R12 = auto()


class RobotState(Enum):
    NONE = auto()
    DEFAULT = auto()
    BOTH = auto()
    ALGAE_GRAB = auto()
    ALGAE_PLACE = auto()
    CORAL_GRAB = auto()
    CORAL_PLACE = auto()
    CLIMB = auto()


class ReefScape(Subsystem):
    # Singleton Instance
    _instance = None

    @classmethod
    def getInstance(cls):
        if cls._instance is None:
            cls._instance = ReefScape()
        return cls._instance

    # Robot information variables
    _current_mode: RobotMode = RobotMode.NONE
    _current_region: RobotRegion = RobotRegion.NONE
    _current_state: RobotState = RobotState.NONE

    # Coral Variables
    _current_source_side: SourceSide = SourceSide.AUTO
    _current_select: SourceSelect = SourceSelect.OUTER
    _current_height: ReefTarget = ReefTarget.L2
    _current_reef_side: ReefSide = ReefSide.R1

    __log: NetworkTable = NetworkTableInstance.getDefault().getTable("/")

    def __init__(self):
        self.setGetRobotPose( lambda: Pose2d() )
        self.setHasCoral(lambda: False)
        self.setHasAlgae(lambda: False)
        self.elevatorAtPosition(lambda: False)

        # Logging for the robot
        self.__log.putString("RobotMode", str(self._current_mode))
        self.__log.putString("RobotRegion", str(self._current_region))
        self.__log.putString("RobotState", str(self._current_state))

        # Logging for the source
        self.__log.putString("SourceSide", str(self._current_source_side))
        self.__log.putString("SourceSelect", str(self._current_select))

        # Logging for the reef
        self.__log.putString("ReefHeight", str(self._current_height))
        self.__log.putString("ReefSide", str(self._current_reef_side))

    def periodic(self) -> None:
        match self.getState():
            case RobotState.NONE:
                pass
            case RobotState.DEFAULT:
                if self.getHasCoral() and self.getHasAlgae():
                    self.setState(RobotState.BOTH)
                if self.getHasCoral():
                    self.setState(RobotState.CORAL_PLACE)
                if self.getHasAlgae():
                    self.setState(RobotState.ALGAE_PLACE)
            case RobotState.ALGAE_GRAB:
                if self.getHasAlgae():
                    self.setState(RobotState.DEFAULT)
            case RobotState.ALGAE_PLACE:
                if not self.getHasAlgae():
                    self.setState(RobotState.DEFAULT)
            case RobotState.CORAL_GRAB:
                if self.getHasCoral():
                    self.setState(RobotState.DEFAULT)
            case RobotState.CORAL_PLACE:
                if not self.getHasCoral():
                    self.setState(RobotState.DEFAULT)
            case RobotState.CLIMB:
                pass
            case RobotState.BOTH:
                if not self.getHasCoral() or not self.getHasAlgae():
                    self.setState(RobotState.DEFAULT)

    def getState(self):
        return self._current_state

    def setState(self, state: RobotState):
        self._current_state = state
        self.__log.putString("RobotState", str(state))

    def setNextSide(self):
        posInt = int( self._current_reef_side.name[1::] )
        if posInt >= 12:
            posInt = 0
        posInt = posInt + 1
        newPos = f"R{posInt}"
        self.setTarget( ReefSide._member_map_[ newPos ] )
        #self._current_reef_side = ReefSide._member_map_[ newPos ]
        #self.__log.putString("ReefSide", str(self._current_reef_side))

    def setNextHeight(self):
        posInt = int( self._current_height.name[1::] )
        if posInt >= 4:
            posInt = 0
        posInt = posInt + 1
        newPos = f"L{posInt}"
        self.setHeight( ReefTarget._member_map_[ newPos ] )

    @classmethod
    def getTarget(self):
        """
        Returns ONE through TWELVE
        """
        return self._current_reef_side

    @classmethod
    def setTarget(self, side: ReefSide):
        """
        Sets the target to ONE through TWELVE
        """
        self._current_reef_side = side
        self.__log.putString("ReefSide", str(side))

    @classmethod
    def setHeight(self, height: ReefTarget):
        """
        Sets the height to L1, L2, L3, or R4
        """
        self._current_height = height
        self.__log.putString("ReefHeight", str(height))

    @classmethod
    def getHeight(self):
        """
        Returns L1, L2, L3, or R4
        """
        return self._current_height

    def getMode(self):
        """
        Returns either TEST, COMPETITION, DEMO, or NONE
        """
        return self._current_mode

    def setMode(self, mode: RobotMode):
        self._current_mode = mode
        self.__log.putString("RobotMode", str(mode))

    def getRegion(self):
        """
        Returns either ALGAE, BARGE, REEF, or SOURCE
        """
        return self._current_region

    @classmethod
    def getSourceSide(self):
        """
        Returns either LEFT, RIGHT, or AUTO
        """
        return self._current_source_side
    
    @classmethod
    def getSourceSelect(self):
        """
        Returns either OUTER, INNER, or MIDDLE
        """
        return self._current_select

    @classmethod
    def setSourceSide(self, side: SourceSide):
        self._current_source_side = side
        self.__log.putString("SourceSide", str(side))

    @classmethod
    def setSourceSelect(self, select: SourceSelect):
        self._current_select = select
        self.__log.putString("SourceSelect", str(select))

    def changeSourceSelect(self):
        match self._current_select:
            case SourceSelect.OUTER:
                self._current_select = SourceSelect.MIDDLE
            case SourceSelect.MIDDLE:
                self._current_select = SourceSelect.INNER
            case SourceSelect.INNER:
                self._current_select = SourceSelect.OUTER
        self.__log.putString("SourceSelect", str(self._current_select))

    def changeSourceSide(self):
        match self._current_source_side:
            case SourceSide.LEFT:
                self._current_source_side = SourceSide.RIGHT
            case SourceSide.RIGHT:
                self._current_source_side = SourceSide.LEFT
        self.__log.putString("SourceSide", str(self._current_source_side))

    # Getters and Setters for what the robot has, and where the elevator is
    def setGetRobotPose(self, getPose: Callable[[], Pose2d]):
        self.__getRobotPose = getPose

    def setHasCoral(self, hasCoral: Callable[[], bool]):
        self.__hasCoral = hasCoral

    def setHasAlgae(self, hasAlgae: Callable[[], bool]):
        self.__lamdaGetHasAlgae = hasAlgae

    def elevatorAtPosition(self, atPosition: Callable[[], bool]):
        self.__lamdaElevatorAtPosition = atPosition

    # Lambdas
    def getRobotPose(self) -> Pose2d:
        return self.__getRobotPose()

    def getHasCoral(self):
        return self.__hasCoral()

    def getHasAlgae(self):
        return self.__lamdaGetHasAlgae()

    def getElevatorAtPosition(self):
        return self.__lamdaElevatorAtPosition()

    #@classmethod
    def getReefPose(self) -> Pose2d:
        return ReefscapePositions.Reef._member_map_[ self.getTarget().name ].value
    
    #@classmethod
    def getSourcePose(self, side:SourceSide = None ) -> Pose2d:
        p = self.getRobotPose()
        if side == None:
            return self.getSourcePose( self.getSourceSide() )
        elif side == SourceSide.AUTO:
            match DriverStation.getAlliance():
                case DriverStation.Alliance.kRed:
                    if ReefspaceField.kRed.LEFT_FIELD.value.contains( p.translation() ):
                        return self.getSourcePose( SourceSide.LEFT )
                    elif ReefspaceField.kRed.RIGHT_FIELD.value.contains( p.translation() ):
                        return self.getSourcePose( SourceSide.RIGHT )
                case DriverStation.Alliance.kBlue:
                    if ReefspaceField.kBlue.LEFT_FIELD.value.contains( p.translation() ):
                        return self.getSourcePose( SourceSide.LEFT )
                    elif ReefspaceField.kBlue.RIGHT_FIELD.value.contains( p.translation() ):
                        return self.getSourcePose( SourceSide.RIGHT )
                case _:
                    pass
        else:
            pos = self.getSourceSelect()
            match side:
                case SourceSide.LEFT:
                    return ReefscapePositions.Source.Left._member_map_[ pos.name ].value
                case SourceSide.RIGHT:
                    return ReefscapePositions.Source.Right._member_map_[ pos.name ].value
                case _:
                    pass
        return p

