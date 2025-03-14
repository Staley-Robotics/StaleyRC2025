from commands2 import Command

from subsystems.Elevator import Elevator

from typing import Callable


class ElevatorByStick(Command):
    maxInchesPerSec:float = 20.00

    def __init__(self, elevatorSubsystem: Elevator, getMovement: Callable[[], float]):
        self.__elevator = elevatorSubsystem
        self.__getMovement = getMovement
        self.setName("ElevatorByStick")
        self.addRequirements(self.__elevator)

    def initialize(self):
        pass

    def execute(self):
        pos = self.__elevator.getSetpoint()
        move = self.__getMovement() * ( self.maxInchesPerSec * 0.02 )
        self.__elevator.setSetpoint( pos + move )

    def isFinished(self):
        return False

    def runsWhenDisabled(self):
        return False

