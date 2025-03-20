from typing import Callable
from wpilib import Timer

from commands2 import Command

from subsystems import CoralManipulatorWheel

class CoralHold(Command):
    def __init__(self, coralWheelSys:CoralManipulatorWheel) -> None:
        self.wheelSys = coralWheelSys

        self.timer = Timer()

        self.addRequirements(coralWheelSys)
        self.setName( f'{self.__class__.__name__}' )
    
    def initialize(self):
        # self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.OUT )
        ...

    
    def execute(self):
        if not self.wheelSys.hasCoral():
            self.timer.start()
            self.wheelSys.setSpeed(CoralManipulatorWheel.WheelSpeeds.IN)

    def end(self, interrupted):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
    
    def isFinished(self):
        return self.wheelSys.hasCoral() or self.timer.get() >= 1
