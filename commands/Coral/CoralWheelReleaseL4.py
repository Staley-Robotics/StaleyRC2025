from typing import Callable

from commands2 import Command

from subsystems import CoralManipulatorWheel

class CoralWheelReleaseL4(Command):
    def __init__(self, coralWheelSys:CoralManipulatorWheel) -> None:
        self.wheelSys = coralWheelSys

        self.addRequirements(coralWheelSys)
        self.setName( f'{self.__class__.__name__}' )
    
    def initialize(self):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
    
    def execute(self):
        pass
    
    def end(self, interrupted):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
        self.wheelSys.has_coral = False
    
    def isFinished(self):
        return not self.wheelSys.hasCoral()
