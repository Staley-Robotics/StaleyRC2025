from commands2 import Command

from subsystems import Elevator, CoralManipulatorPivot, CoralPivotPositions

from typing import Callable

from ntcore.util import ntproperty


class ElevatorResync(Command):
        
    def __init__(self, elevatorSubsystem: Elevator, coralSubsystem:CoralManipulatorPivot):
        self.__elevator = elevatorSubsystem
        self.__coral = coralSubsystem

        self.setName("ElevatorResync")
        self.addRequirements(elevatorSubsystem, coralSubsystem)

    def initialize(self):
        if self.__coral.getPosition() < -60:
            self.__coral.setSetpoint( -60 )

    def execute(self):
        if self.__coral.atSetpoint():
            self.__elevator.setOpenControl( -0.5 )
    
    def end(self, interrupted):
        self.__elevator.resetPosition()
        self.__elevator.stop()

    def isFinished(self):
        return self.__elevator.getSwitchesState() == -1

    def runsWhenDisabled(self):
        return False
