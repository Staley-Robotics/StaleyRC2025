from enum import Enum, auto
from typing import Callable

from commands2 import Subsystem
from ntcore import NetworkTable, NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d


class ReefScapePositions(Enum):
    class Reef(Enum):
        R1: Pose2d = Pose2d(4.998, 5.199, Rotation2d.fromDegrees(-120))
        R2: Pose2d = Pose2d(5.263, 5.053, Rotation2d.fromDegrees(-120))
        R3: Pose2d = Pose2d(5.758, 4.194, Rotation2d.fromDegrees(180))
        R4: Pose2d = Pose2d(4.763, 3.852, Rotation2d.fromDegrees(180))
        R5: Pose2d = Pose2d(5.272, 3, Rotation2d.fromDegrees(120))
        R6: Pose2d = Pose2d(4.998, 2.830, Rotation2d.fromDegrees(120))
        R7: Pose2d = Pose2d(4, 2.829, Rotation2d.fromDegrees(60))
        R8: Pose2d = Pose2d(3.71, 3, Rotation2d.fromDegrees(60))
        R9: Pose2d = Pose2d(3.2, 3.86, Rotation2d.fromDegrees(0))
        R10: Pose2d = Pose2d(3.2, 4.187, Rotation2d.fromDegrees(0))
        R11: Pose2d = Pose2d(3.716, 5.058, Rotation2d.fromDegrees(-60))
        R12: Pose2d = Pose2d(3.976, 5.212, Rotation2d.fromDegrees(-60))

    class Source(Enum):
        class LEFT(Enum):
            OUTER: Pose2d = Pose2d(1.7, 7.413, Rotation2d.fromDegrees(-27))
            MIDDLE: Pose2d = Pose2d(1.2, 7.051, Rotation2d.fromDegrees(-27))
            INNER: Pose2d = Pose2d(0.660, 6.681, Rotation2d.fromDegrees(-27))

        class RIGHT(Enum):
            OUTER: Pose2d = Pose2d(1.459, 0.79, Rotation2d.fromDegrees(46.5))
            MIDDLE: Pose2d = Pose2d(1.202, 1.029, Rotation2d.fromDegrees(46.5))
            INNER: Pose2d = Pose2d(0.734, 1.340, Rotation2d.fromDegrees(46.5))

class RobotMode(Enum):
    TEST = auto()
    COMPETITION = auto()
    DEMO = auto()
    NONE = auto()


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
    OUTER = auto()
    INNER = auto()
    MIDDLE = auto()


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
    _current_source_side: SourceSide = SourceSide.LEFT
    _current_select: SourceSelect = SourceSelect.OUTER
    _current_height: ReefTarget = ReefTarget.L2
    _current_reef_side: ReefSide = ReefSide.R1

    __log: NetworkTable = NetworkTableInstance.getDefault().getTable("/ReefScapeState")

    def __init__(self):
        self.setHasCoral(lambda: False)
        self.setHasAlgae(lambda: False)
        self.setElevatorAtPosition(lambda: False)

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

    def setNextReefSide(self):
        match self._current_reef_side:
            case ReefSide.R1:
                self._current_reef_side = ReefSide.R2
            case ReefSide.R2:
                self._current_reef_side = ReefSide.R3
            case ReefSide.R3:
                self._current_reef_side = ReefSide.R4
            case ReefSide.R4:
                self._current_reef_side = ReefSide.R5
            case ReefSide.R5:
                self._current_reef_side = ReefSide.R6
            case ReefSide.R6:
                self._current_reef_side = ReefSide.R7
            case ReefSide.R7:
                self._current_reef_side = ReefSide.R8
            case ReefSide.R8:
                self._current_reef_side = ReefSide.R9
            case ReefSide.R9:
                self._current_reef_side = ReefSide.R10
            case ReefSide.R10:
                self._current_reef_side = ReefSide.R11
            case ReefSide.R11:
                self._current_reef_side = ReefSide.R12
            case ReefSide.R12:
                self._current_reef_side = ReefSide.R1
        self.__log.putString("ReefSide", str(self._current_reef_side))

    def setNextHeight(self):
        match self._current_height:
            case ReefTarget.L1:
                self._current_height = ReefTarget.L2
            case ReefTarget.L2:
                self._current_height = ReefTarget.L3
            case ReefTarget.L3:
                self._current_height = ReefTarget.L4
            case ReefTarget.L4:
                self._current_height = ReefTarget.L1
        self.__log.putString("ReefHeight", str(self._current_height))

    @classmethod
    def getTarget(self):
        """
        Returns R1 through R12
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

    def getSourceSide(self):
        """
        Returns either LEFT, RIGHT, or AUTO
        """
        return self._current_source_side

    def getSourceSelect(self):
        """
        Returns either OUTER, INNER, or MIDDLE
        """
        return self._current_select

    def setSourceSide(self, side: SourceSide):
        self._current_source_side = side
        self.__log.putString("SourceSide", str(side))

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
            case SourceSide.AUTO:
                self._current_source_side = SourceSide.LEFT
        self.__log.putString("SourceSide", str(self._current_source_side))

    # Getters and Setters for what the robot has, and where the elevator is

    def setHasCoral(self, hasCoral: Callable[[], bool]):
        self.__hasCoral = hasCoral
        self.__log.putBoolean("HasCoral", hasCoral())

    def setHasAlgae(self, hasAlgae: Callable[[], bool]):
        self.__lamdaGetHasAlgae = hasAlgae

    def setElevatorAtPosition(self, atPosition: Callable[[], bool]):
        self.__lamdaElevatorAtPosition = atPosition

    def getHasCoral(self):
        return self.__hasCoral()

    def getHasAlgae(self):
        return self.__lamdaGetHasAlgae()

    def getElevatorAtPosition(self):
        return self.__lamdaElevatorAtPosition()

    @classmethod
    def getReefPose(self) -> Pose2d:
        instance = self.getInstance()
        return ReefScapePositions.Reef._member_map_[ instance._current_reef_side.name ].value

    @classmethod
    def getCoralPose(self) -> Pose2d:
        instance = self.getInstance()
        return ReefScapePositions.Source._member_map_[ instance._current_source_side.name ]._member_map_[ self._current_select.name ].value

    @classmethod
    def getTargetRotation(self) -> Rotation2d:
        instance = self.getInstance()
        if instance.getHasCoral():
            return self.getReefPose().rotation()
        else:
            side_name = instance._current_source_side.name
            select_name = instance._current_select.name
            if side_name == 'LEFT':
                return ReefScapePositions.Source.LEFT._member_map_[select_name].value.rotation()
            elif side_name == 'RIGHT':
                return ReefScapePositions.Source.RIGHT._member_map_[select_name].value.rotation()
            else:
                # Handle AUTO or other cases
                return ReefScapePositions.Source.LEFT._member_map_[select_name].value.rotation()  # Default fallback