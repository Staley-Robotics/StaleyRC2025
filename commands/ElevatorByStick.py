from commands2 import Command

from subsystems.Elevator import Elevator

from typing import Callable


class ElevatorByStick(Command):
    def __init__(self, elevatorSubsystem: Elevator, pos: Callable[[], float]):
        self.__elevator = elevatorSubsystem
        self.getPos = pos
        self.__position = 0.0


        self.setName("ElevatorByStick")
        self.addRequirements(self.__elevator)

    def initialize(self):
        pass

    def execute(self):
        self.__position = self.scaleToRange(self.getPos())
        self.__elevator.setSetpoint(self.__position)
        print(f"Setpoint: {self.__position}")

    def isFinished(self):
        return False

    def runsWhenDisabled(self):
        return False

    def scaleToRange(self, value):
        fromLow = -1
        fromHigh = 1
        toLow = 0
        toHigh = 1
        return (value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow
