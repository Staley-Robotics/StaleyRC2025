from enum import Enum, auto
from typing import Callable

from commands2 import Subsystem
from ntcore import NetworkTable, NetworkTableInstance


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
    R4 = auto()


class ReefSide(Enum):
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()


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
    _current_reef_side: ReefSide = ReefSide.ONE

    __log: NetworkTable = NetworkTableInstance.getDefault().getTable("/")

    def __init__(self):
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
        match self._current_reef_side:
            case ReefSide.ONE:
                self._current_reef_side = ReefSide.TWO
            case ReefSide.TWO:
                self._current_reef_side = ReefSide.THREE
            case ReefSide.THREE:
                self._current_reef_side = ReefSide.FOUR
            case ReefSide.FOUR:
                self._current_reef_side = ReefSide.FIVE
            case ReefSide.FIVE:
                self._current_reef_side = ReefSide.SIX
            case ReefSide.SIX:
                self._current_reef_side = ReefSide.SEVEN
            case ReefSide.SEVEN:
                self._current_reef_side = ReefSide.EIGHT
            case ReefSide.EIGHT:
                self._current_reef_side = ReefSide.NINE
            case ReefSide.NINE:
                self._current_reef_side = ReefSide.TEN
            case ReefSide.TEN:
                self._current_reef_side = ReefSide.ELEVEN
            case ReefSide.ELEVEN:
                self._current_reef_side = ReefSide.TWELVE
            case ReefSide.TWELVE:
                self._current_reef_side = ReefSide.ONE
        self.__log.putString("ReefSide", str(self._current_reef_side))

    def setNextHeight(self):
        match self._current_height:
            case ReefTarget.L1:
                self._current_height = ReefTarget.L2
            case ReefTarget.L2:
                self._current_height = ReefTarget.L3
            case ReefTarget.L3:
                self._current_height = ReefTarget.R4
            case ReefTarget.R4:
                self._current_height = ReefTarget.L1
        self.__log.putString("ReefHeight", str(self._current_height))

    def getTarget(self):
        """
        Returns ONE through TWELVE
        """
        return self._current_reef_side

    def setTarget(self, side: ReefSide):
        """
        Sets the target to ONE through TWELVE
        """
        self._current_reef_side = side
        self.__log.putString("ReefSide", str(side))

    def setHeight(self, height: ReefTarget):
        """
        Sets the height to L1, L2, L3, or R4
        """
        self._current_height = height
        self.__log.putString("ReefHeight", str(height))

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
        self.__log.putString("SourceSide", str(self._current_source_side))

    # Getters and Setters for what the robot has, and where the elevator is

    def setHasCoral(self, hasCoral: Callable[[], bool]):
        self.__hasCoral = hasCoral

    def setHasAlgae(self, hasAlgae: Callable[[], bool]):
        self.__lamdaGetHasAlgae = hasAlgae

    def elevatorAtPosition(self, atPosition: Callable[[], bool]):
        self.__lamdaElevatorAtPosition = atPosition

    # Lambdas

    def getHasCoral(self):
        return self.__hasCoral

    def getHasAlgae(self):
        return self.__lamdaGetHasAlgae()

    def getElevatorAtPosition(self):
        return self.__lamdaElevatorAtPosition()
