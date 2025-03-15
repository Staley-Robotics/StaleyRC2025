from commands2 import Command

from subsystems.Elevator import Elevator

from typing import Callable

from ntcore.util import ntproperty


class ElevatorByStick(Command):
    
    controlSpeed = ntproperty('/Settings/Elevator/ControlSpeed', 1, persistent=True)
    
    def __init__(self, elevatorSubsystem: Elevator, pos: Callable[[], float]):
        self.__elevator = elevatorSubsystem
        self.getPos = pos

        self.setPos = Elevator.ElevatorPositions.BOTTOM


        self.setName("ElevatorByStick")
        self.addRequirements(self.__elevator)

    def initialize(self):
        self.getPos = self.__elevator.getSetpoint()

    def execute(self):
        self.setPos = max(min(self.setPos + self.getPos() * self.controlSpeed, Elevator.ElevatorPositions.TOP),Elevator.ElevatorPositions.BOTTOM)
        self.__elevator.setSetpoint(self.setPos)
        # self.__elevator.setSetpoint(self.scaleToRange(self.getPos()))

    def isFinished(self):
        return False

    def runsWhenDisabled(self):
        return False

    def scaleToRange(self, value):
        # fromLow = -1
        # fromHigh = 1
        # toLow = 0
        # toHigh = 1
        # return (value - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow
        minimum = Elevator.ElevatorPositions.BOTTOM
        maximum = Elevator.ElevatorPositions.TOP
        value = (value + 1)/2 # value from [-1,1] -> [0,1]
        return minimum + (value * (maximum - minimum)) # value from [0,1] -> [minimum, maximum]
