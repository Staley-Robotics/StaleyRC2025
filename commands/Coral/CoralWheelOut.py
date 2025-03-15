from typing import Callable
from wpilib import Timer

from commands2 import Command

from subsystems import CoralManipulatorWheel

class CoralWheelOut(Command):
    def __init__(self, coralWheelSys:CoralManipulatorWheel) -> None:
        self.wheelSys = coralWheelSys
        self.timer = Timer()

        self.addRequirements(coralWheelSys)
        self.setName( f'{self.__class__.__name__}' )
    
    def initialize(self):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.OUT )
    
    def execute(self):
        if not self.wheelSys.hasCoral() and not self.timer.isRunning():
            self.timer.reset()
            self.timer.start()

    def end(self, interrupted):
        self.wheelSys.setSpeed( CoralManipulatorWheel.WheelSpeeds.STOP )
        self.timer.stop()
        self.timer.reset()
    
    def isFinished(self):
        return not self.wheelSys.hasCoral() and self.timer.hasElapsed(0.5) # TODO: This is probably wrong
