from commands2 import Command

from subsystems.Elevator import Elevator

from typing import Callable

from ntcore.util import ntproperty


class ElevatorResync(Command):
        
    def __init__(self, elevatorSubsystem: Elevator):
        self.__elevator = elevatorSubsystem

        self.setName("ElevatorResync")
        self.addRequirements(self.__elevator)

    def initialize(self):
        ...

    def execute(self):
        self.__elevator.setOpenControl( -0.5 )
    
    def end(self):
        self.__elevator.resetPosition()
        self.__elevator.stop()

    def isFinished(self):
        return self.__elevator.getSwitchesState() == -1

    def runsWhenDisabled(self):
        return False
