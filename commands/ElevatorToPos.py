from typing import Callable
from commands2 import Command

from subsystems.Elevator import Elevator, ElevatorPositions

class ElevatorToPos(Command):
    def __init__(self, elevatorSubsystem: Elevator, getPosition: Callable[[], float]):
        self.__elevator = elevatorSubsystem
        self.__position = getPosition

        self.setName(f"ElevatorToPosition({self.__position()})")
        self.addRequirements(self.__elevator)

    # def initialize(self):
    #     return self.__elevator.setSetpoint(self.__position())

    def execute(self):
        return self.__elevator.setSetpoint( self.__position() )

    def isFinished(self):
        return self.__elevator.atSetpoint()
    
    def runsWhenDisabled(self):
        return False