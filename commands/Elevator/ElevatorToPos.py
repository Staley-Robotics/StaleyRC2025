from commands2 import Command

from subsystems.Elevator import Elevator

class ElevatorToPos(Command):
    def __init__(self, elevatorSubsystem: Elevator, pos: float):
        self.__elevator = elevatorSubsystem
        self.__position = pos

        self.setName(f"ElevatorToPosition({self.__position})")
        self.addRequirements(self.__elevator)

    def initialize(self):
        self.__elevator.setSetpoint(self.__position)

    def isFinished(self):
        return self.__elevator.atSetpoint()
    
    def runsWhenDisabled(self):
        return False